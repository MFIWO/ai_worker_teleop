#!/usr/bin/env python3
#
# Copyright 2026 Config Intelligence Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Tony Lee

"""
Stream physical_ai_server robot state and camera frames into Config Loop.

Modeled on the GR00T-WholeBodyControl Loop sidecar: an OPTIONAL, read-only sink
built on ``loop-sdk`` 0.4.1 (Config Loop v1.1.4). Two independent streams:

  1. Robot state + action -> ``loop_sdk.RobotStepSender`` (gRPC, one ``robot-step``
     source, non-blocking send)
  2. Camera media -> an in-process RTSP/RTP-JPEG server that Loop PULLS from,
     advertised over gRPC via ``loop_sdk.SourceProducer``.

Why an RTSP server at all? loop-sdk has no ``send_camera_frame()`` API: Loop
pulls camera media from an RTSP/RTP endpoint and the SDK only advertises where.
The RTP/JPEG packetization and RTSP request handling below are VENDORED from the
loop-sdk e2e fixture (``loop-sdk/e2e/send_simulated_sources.py``), the only
camera transport proven to interoperate with Loop; the only change is the frame
source: the fixture loops a fixed frame list, this server sends the latest live
frame. ``CompressedImage`` topics already carry JPEG bytes, so frames are
packetized directly without a decode / re-encode round trip.

Threading / data flow (cameras)::

    ROS callback thread                         background threads
    -------------------                         ------------------
    CompressedImage.data --set_jpeg()-->  [latest raw slot, lock]
                                              |  (per-camera encoder thread)
                                              v  parse_jpeg_for_rtp / cv2 fallback
                                        [latest JpegRtpFrame slot, lock]
                                              |  RtspLiveServer (camera fps, asyncio)
                                              v
                                        RTP/JPEG over RTSP/TCP  --> Loop pulls

The ROS thread only swaps a bytes reference; JPEG parsing and all network I/O
happen off-thread, so a slow or dead Loop consumer can never stall the ROS
executor. ``loop_sdk`` is imported lazily so the rest of ``physical_ai_server``
keeps working without it. This module has no ROS dependency; run it directly for
a smoke test against a Loop instance (``python3 -m physical_ai_server.loop_streamer``).
"""

import asyncio
import contextlib
from dataclasses import dataclass, field
import logging
import random
import socket
import struct
import threading
import time
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlsplit

logger = logging.getLogger('loop_streamer')

# ============================================================================
# Robot state / action mapping
# ============================================================================
# The layout mirrors what DataManager records into a LeRobot dataset, but keeps
# the per-group structure the robot config defines under ``joint_order``:
#
#     <robot_type>.observation.<group>.joint_position   <- follower JointState
#     <robot_type>.action.<group>.joint_position        <- leader JointTrajectory
#     <robot_type>.observation.<group>.velocity         <- follower Odometry (mobile)
#     <robot_type>.action.<group>.velocity              <- leader Twist      (mobile)
#
# <group> is the joint_order key without its ``leader`` prefix (``leader_left``
# -> ``left``; a single ``leader`` group -> ``arm``). Grippers are ordinary joints
# inside the arm groups. Missing readings are sent as ``None`` ("no reading this
# tick") instead of zeros; the channel layout itself is declared up front so it
# never depends on which messages have arrived.

JOINT_CHANNEL = 'joint_position'
MOBILE_CHANNEL = 'velocity'
MOBILE_AXES = ('linear_x', 'linear_y', 'angular_z')
JOINT_ORDER_PREFIX = 'joint_order.'
LEADER_PREFIX = 'leader'
DEFAULT_GROUP_LABEL = 'arm'

ChannelValue = Optional[List[float]]


@dataclass(frozen=True)
class JointGroup:
    """One ``joint_order`` group of the robot config."""

    name: str                      # config name, e.g. 'leader_left'
    label: str                     # channel label, e.g. 'left'
    joint_names: Tuple[str, ...]   # ordered joint names (or mobile axis names)
    mobile: bool                   # Twist / Odometry group instead of joints

    @property
    def channel(self) -> str:
        return MOBILE_CHANNEL if self.mobile else JOINT_CHANNEL


@dataclass(frozen=True)
class ChannelInfo:
    """Metadata for one declared channel, independent of loop-sdk types."""

    key: str
    label: str
    unit: str
    group: str


def group_label(name: str) -> str:
    """Return the channel label of a ``joint_order`` group name."""
    label = name
    if label.startswith(JOINT_ORDER_PREFIX):
        label = label[len(JOINT_ORDER_PREFIX):]
    if label.startswith(LEADER_PREFIX):
        label = label[len(LEADER_PREFIX):].lstrip('_')
    return label or DEFAULT_GROUP_LABEL


def is_mobile_group(name: str) -> bool:
    """Return whether a group carries a mobile base command (Twist / Odometry)."""
    return 'mobile' in name.lower()


def build_groups(joint_order: Mapping[str, Sequence[str]]) -> Tuple[JointGroup, ...]:
    """
    Build the joint groups from the loaded ``joint_order`` parameters.

    ``joint_order`` keys may be plain group names or the ``joint_order.<name>``
    form produced by ``load_parameters``. Insertion order is preserved so the
    declared channel layout follows the robot config.
    """
    groups = []
    for raw_name, joint_names in joint_order.items():
        name = raw_name[len(JOINT_ORDER_PREFIX):] if raw_name.startswith(JOINT_ORDER_PREFIX) \
            else raw_name
        mobile = is_mobile_group(name)
        names = tuple(MOBILE_AXES) if mobile else tuple(
            str(joint) for joint in joint_names if str(joint))
        if not names:
            raise ValueError(f"joint group '{name}' has no joints")
        groups.append(JointGroup(
            name=name,
            label=group_label(name),
            joint_names=names,
            mobile=mobile,
        ))
    return tuple(groups)


def as_float_list(values: Iterable[float]) -> List[float]:
    """
    Coerce a numpy array / list / tuple into a clean ``list[float]``.

    loop-sdk treats a list made only of numbers as ONE vector channel; a stray
    ``None`` inside would flip it into indexed scalars and silently change the
    layout, so vectors are always emitted as clean float lists.
    """
    return [float(value) for value in values]


class RobotStepMapper:
    """Build fixed-layout ``robot-step`` payloads for one robot type."""

    def __init__(self, robot_type: str, groups: Sequence[JointGroup]):
        if not robot_type:
            raise ValueError('robot_type must not be empty')
        if not groups:
            raise ValueError('at least one joint group is required')
        self._robot_type = robot_type
        self._groups = tuple(groups)
        labels = [group.label for group in self._groups]
        duplicates = sorted({label for label in labels if labels.count(label) > 1})
        if duplicates:
            raise ValueError(f'joint groups map to duplicate channel labels: {duplicates}')

    @property
    def robot_type(self) -> str:
        return self._robot_type

    @property
    def groups(self) -> Tuple[JointGroup, ...]:
        return self._groups

    def observation_key(self, group: JointGroup) -> str:
        return f'{self._robot_type}.observation.{group.label}.{group.channel}'

    def action_key(self, group: JointGroup) -> str:
        return f'{self._robot_type}.action.{group.label}.{group.channel}'

    def channel_infos(self) -> List[ChannelInfo]:
        """Return the fixed channel layout, observations first then actions."""
        infos = []
        for group in self._groups:
            infos.append(self._channel_info(self.observation_key(group), group, 'observation'))
        for group in self._groups:
            infos.append(self._channel_info(self.action_key(group), group, 'action'))
        return infos

    def channel_keys(self) -> List[str]:
        return [info.key for info in self.channel_infos()]

    def build_payload(
        self,
        follower_positions: Optional[Mapping[str, float]] = None,
        leader_positions: Optional[Mapping[str, Mapping[str, float]]] = None,
        mobile_state: Optional[Sequence[float]] = None,
        mobile_command: Optional[Sequence[float]] = None,
    ) -> Dict[str, ChannelValue]:
        """
        Build one payload from the latest data of each source.

        ``follower_positions`` maps every follower joint name to its measured
        position (all follower JointState topics merged). ``leader_positions``
        maps a group name to ``{joint_name: commanded position}``. The mobile
        vectors are ``[linear_x, linear_y, angular_z]`` from Odometry (state) and
        Twist (command). Any group whose reading is unavailable or incomplete
        yields ``None`` for that channel.
        """
        payload: Dict[str, ChannelValue] = {}
        for group in self._groups:
            if group.mobile:
                payload[self.observation_key(group)] = self._mobile_vector(mobile_state)
            else:
                payload[self.observation_key(group)] = self._ordered_vector(
                    follower_positions, group.joint_names)
        for group in self._groups:
            if group.mobile:
                payload[self.action_key(group)] = self._mobile_vector(mobile_command)
            else:
                positions = leader_positions.get(group.name) if leader_positions else None
                payload[self.action_key(group)] = self._ordered_vector(
                    positions, group.joint_names)
        return payload

    def missing_follower_joints(self, follower_positions: Mapping[str, float]) -> List[str]:
        """Return the joint names the follower does not report for any group."""
        missing = []
        for group in self._groups:
            if group.mobile:
                continue
            missing.extend(name for name in group.joint_names if name not in follower_positions)
        return missing

    @staticmethod
    def _ordered_vector(
        positions: Optional[Mapping[str, float]],
        joint_names: Sequence[str],
    ) -> ChannelValue:
        if positions is None:
            return None
        try:
            return [float(positions[name]) for name in joint_names]
        except KeyError:
            return None

    @staticmethod
    def _mobile_vector(values: Optional[Sequence[float]]) -> ChannelValue:
        if values is None:
            return None
        vector = as_float_list(values)
        if len(vector) != len(MOBILE_AXES):
            return None
        return vector

    @staticmethod
    def _channel_info(key: str, group: JointGroup, role: str) -> ChannelInfo:
        return ChannelInfo(
            key=key,
            label=', '.join(group.joint_names),
            unit='' if group.mobile else 'rad',
            group=f'{role}.{group.label}',
        )


# ============================================================================
# Robot state streaming
# ============================================================================
DEFAULT_LOOP_ADDR = 'localhost:50051'
DEFAULT_SOURCE_KEY = 'robot-step'
DEFAULT_ACTION_SPACE = 'joint_position'


class LoopRobotStreamer:
    """Wrap ``loop_sdk.RobotStepSender`` for the physical_ai_server teleop loop."""

    def __init__(
        self,
        loop_addr: str,
        mapper: RobotStepMapper,
        *,
        source_key: str = DEFAULT_SOURCE_KEY,
        action_space: str = DEFAULT_ACTION_SPACE,
        connect_timeout_s: float = 5.0,
    ) -> None:
        self._loop_addr = loop_addr
        self._mapper = mapper
        self._source_key = source_key
        self._action_space = action_space
        self._connect_timeout_s = connect_timeout_s
        self._sender = None
        self._send_failed_once = False
        self.sent_count = 0

    @property
    def mapper(self) -> RobotStepMapper:
        return self._mapper

    @property
    def source_key(self) -> str:
        return self._source_key

    def connect(self) -> None:
        """
        Declare the source and FAIL FAST when Config Loop is unreachable.

        ``RobotStepSender.connect()`` / ``declare()`` never block: the gRPC
        channel is lazy and does not raise on an unreachable address. So the
        layout is declared and then ``stats().connected`` is polled until a
        deadline. This confirms the channel opened, not that Loop accepted the
        registration.
        """
        from loop_sdk import ChannelSpec, RobotConfigOptions, RobotStepSender

        options = RobotConfigOptions(
            action_space=(self._action_space,),
            robot_type=(self._mapper.robot_type,),
        )
        sender = RobotStepSender(
            self._loop_addr,
            self._source_key,
            name=self._source_key,
            options=options,
        )
        sender.connect()
        sender.declare(self.channel_specs(ChannelSpec))
        self._sender = sender

        deadline = time.monotonic() + self._connect_timeout_s
        while time.monotonic() < deadline:
            if self.is_connected():
                logger.info('[loop] robot-step source %r connected to %s',
                            self._source_key, self._loop_addr)
                return
            time.sleep(0.1)
        self.close()
        raise ConnectionError(
            f'[loop] could not reach Config Loop at {self._loop_addr} within '
            f'{self._connect_timeout_s:g}s (robot-step). Is Loop running?')

    def channel_specs(self, channel_spec_type) -> list:
        """Build loop-sdk ``ChannelSpec`` objects for the mapper's fixed layout."""
        return [
            channel_spec_type(key=info.key, label=info.label, unit=info.unit, group=info.group)
            for info in self._mapper.channel_infos()
        ]

    def is_connected(self) -> bool:
        if self._sender is None:
            return False
        try:
            return bool(getattr(self._sender.stats(), 'connected', False))
        except Exception:
            return False

    def send(self, payload: Mapping[str, ChannelValue]) -> bool:
        """
        Publish one tick; never raises into the caller's timer loop.

        The SDK stamps the publish time itself so every Loop source shares one
        clock. Returns ``True`` when the SDK accepted the payload.
        """
        if self._sender is None:
            return False
        try:
            accepted = bool(self._sender.send(dict(payload)))
        except Exception as error:
            if not self._send_failed_once:
                logger.warning('[loop] robot-step send failed (will keep trying silently): %s',
                               error)
                self._send_failed_once = True
            return False
        if accepted:
            self.sent_count += 1
        return accepted

    def stats(self):
        if self._sender is None:
            return None
        with contextlib.suppress(Exception):
            return self._sender.stats()
        return None

    def close(self) -> None:
        sender = self._sender
        self._sender = None
        if sender is not None:
            with contextlib.suppress(Exception):
                sender.disconnect()


# ============================================================================
# Camera streaming
# ============================================================================
# --- VENDORED from loop-sdk/e2e/send_simulated_sources.py (RTP/JPEG core) -----
# Keep these byte-for-byte compatible with the SDK fixture; only the frame source
# (RtspLiveServer._stream reading a LatestFrame) is physical_ai_server-specific.
RTP_CLOCK_RATE = 90_000
RTP_PAYLOAD_TYPE = 26
MAX_RTP_PAYLOAD = 1_200
RTSP_SESSION_ID = 'physical-ai-loop'
RTSP_SERVER_NAME = 'physical-ai-loop-streamer'
SHUTDOWN_TIMEOUT_SECONDS = 3.0
# RTP/JPEG carries width/height as 8px block counts in one byte each.
MAX_RTP_JPEG_DIM = 255 * 8

_SOF_MARKERS = frozenset({0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                          0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF})


@dataclass(frozen=True)
class JpegRtpFrame:
    """One JPEG frame split into the parts an RTP/JPEG packetizer needs."""

    width_blocks: int
    height_blocks: int
    jpeg_type: int
    restart_interval: int
    quantization_tables: bytes
    scan_data: bytes


def is_jpeg(data: bytes) -> bool:
    """Return whether the buffer starts with a JPEG start-of-image marker."""
    return len(data) >= 4 and bytes(data[:2]) == b'\xff\xd8'


def jpeg_dimensions(jpeg: bytes) -> Tuple[int, int]:
    """Return ``(width, height)`` from any JPEG SOF marker without decoding."""
    jpeg = bytes(jpeg)
    if not is_jpeg(jpeg):
        raise ValueError('JPEG frame must start with SOI')
    offset = 2
    while offset + 4 <= len(jpeg):
        if jpeg[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(jpeg) and jpeg[offset] == 0xFF:
            offset += 1
        if offset >= len(jpeg):
            break
        marker = jpeg[offset]
        offset += 1
        if marker in {0x01, 0xD8, 0xD9, *range(0xD0, 0xD8)}:
            continue
        if offset + 2 > len(jpeg):
            break
        segment_length = int.from_bytes(jpeg[offset:offset + 2], 'big')
        if marker in _SOF_MARKERS:
            if offset + 7 > len(jpeg):
                raise ValueError('Malformed JPEG frame header')
            height = int.from_bytes(jpeg[offset + 3:offset + 5], 'big')
            width = int.from_bytes(jpeg[offset + 5:offset + 7], 'big')
            if width <= 0 or height <= 0:
                raise ValueError('JPEG frame header has invalid dimensions')
            return width, height
        if marker == 0xDA:
            break
        offset += segment_length
    raise ValueError('JPEG frame is missing a frame header')


def parse_jpeg_for_rtp(jpeg: bytes) -> JpegRtpFrame:
    """Split a baseline JPEG into RTP/JPEG parts; raises for unsupported JPEGs."""
    if len(jpeg) < 4 or jpeg[:2] != b'\xff\xd8':
        raise ValueError('MJPEG frame must start with SOI')
    offset = 2
    width = height = 0
    jpeg_type = 1
    restart_interval = 0
    quantization_tables: Dict[int, bytes] = {}
    scan_start = 0
    scan_end = len(jpeg)
    while offset < len(jpeg):
        if jpeg[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(jpeg) and jpeg[offset] == 0xFF:
            offset += 1
        if offset >= len(jpeg):
            break
        marker = jpeg[offset]
        offset += 1
        if marker == 0xD9:
            break
        if marker in {0x01, *range(0xD0, 0xD8)}:
            continue
        if offset + 2 > len(jpeg):
            raise ValueError('Malformed JPEG marker length')
        segment_length = int.from_bytes(jpeg[offset:offset + 2], 'big')
        if segment_length < 2 or offset + segment_length > len(jpeg):
            raise ValueError('Malformed JPEG segment')
        segment = jpeg[offset + 2:offset + segment_length]
        offset += segment_length
        if marker == 0xDB:
            quantization_tables.update(_jpeg_quantization_tables(segment))
        elif marker == 0xDD:
            restart_interval = _jpeg_restart_interval(segment)
        elif marker == 0xC0:
            width, height, jpeg_type = _jpeg_frame_header(segment)
        elif marker == 0xDA:
            scan_start = offset
            eoi = jpeg.rfind(b'\xff\xd9')
            scan_end = eoi if eoi > scan_start else len(jpeg)
            break
    if width <= 0 or height <= 0:
        raise ValueError('MJPEG frame is missing baseline dimensions')
    if scan_start <= 0:
        raise ValueError('MJPEG frame is missing scan data')
    width_blocks = (width + 7) // 8
    height_blocks = (height + 7) // 8
    if width_blocks > 255 or height_blocks > 255:
        raise ValueError('MJPEG frame is too large for RTP/JPEG dimensions')
    luma_table = quantization_tables.get(0)
    chroma_table = quantization_tables.get(1, luma_table)
    if luma_table is None or chroma_table is None:
        raise ValueError('MJPEG frame is missing quantization tables')
    if restart_interval > 0:
        jpeg_type += 64
    return JpegRtpFrame(
        width_blocks=width_blocks,
        height_blocks=height_blocks,
        jpeg_type=jpeg_type,
        restart_interval=restart_interval,
        quantization_tables=luma_table + chroma_table,
        scan_data=jpeg[scan_start:scan_end],
    )


def _jpeg_quantization_tables(segment: bytes) -> Dict[int, bytes]:
    tables: Dict[int, bytes] = {}
    offset = 0
    while offset < len(segment):
        table_spec = segment[offset]
        offset += 1
        precision = table_spec >> 4
        table_id = table_spec & 0x0F
        table_size = 64 * (2 if precision else 1)
        if precision != 0:
            raise ValueError('Only 8-bit MJPEG quantization tables are supported')
        if offset + table_size > len(segment):
            raise ValueError('Malformed MJPEG quantization table')
        tables[table_id] = segment[offset:offset + table_size]
        offset += table_size
    return tables


def _jpeg_restart_interval(segment: bytes) -> int:
    if len(segment) != 2:
        raise ValueError('Malformed MJPEG restart interval')
    return int.from_bytes(segment, 'big')


def _jpeg_frame_header(segment: bytes) -> Tuple[int, int, int]:
    if len(segment) < 9:
        raise ValueError('Malformed MJPEG frame header')
    height = int.from_bytes(segment[1:3], 'big')
    width = int.from_bytes(segment[3:5], 'big')
    sampling = segment[7]
    jpeg_type = 0 if sampling == 0x21 else 1
    return width, height, jpeg_type


def rtp_header(sequence: int, timestamp: int, ssrc: int, marker: bool) -> bytes:
    return struct.pack(
        '!BBHII', 0x80, (0x80 if marker else 0) | RTP_PAYLOAD_TYPE, sequence, timestamp, ssrc)


def mjpeg_rtp_payloads(frame: JpegRtpFrame) -> list:
    """Split one frame into RTP/JPEG payloads (header + optional tables + scan)."""
    fragments = []
    offset = 0
    while offset < len(frame.scan_data):
        quantization_header = b''
        if offset == 0:
            quantization_header = b'\x00\x00' + len(frame.quantization_tables).to_bytes(2, 'big')
            quantization_header += frame.quantization_tables
        restart_header = b''
        if frame.restart_interval > 0:
            restart_header = frame.restart_interval.to_bytes(2, 'big') + b'\xff\xff'
        chunk_size = MAX_RTP_PAYLOAD - 8 - len(restart_header) - len(quantization_header)
        if chunk_size <= 0:
            raise ValueError('MJPEG quantization tables are too large for RTP payload')
        chunk = frame.scan_data[offset:offset + chunk_size]
        header = (
            b'\x00' + offset.to_bytes(3, 'big')
            + bytes([frame.jpeg_type, 255, frame.width_blocks, frame.height_blocks])
        )
        fragments.append(header + restart_header + quantization_header + chunk)
        offset += len(chunk)
    return fragments


async def _write_mjpeg_rtp_frame(writer, channel, sequence, timestamp, ssrc, frame) -> int:
    payloads = mjpeg_rtp_payloads(frame)
    for index, payload in enumerate(payloads):
        marker = index == len(payloads) - 1
        rtp = rtp_header(sequence, timestamp, ssrc, marker) + payload
        writer.write(b'$' + bytes([channel]) + struct.pack('!H', len(rtp)) + rtp)
        await writer.drain()
        sequence = (sequence + 1) % 2**16
    return sequence


async def _read_rtsp_request(reader):
    header = await _read_until(reader, b'\r\n\r\n')
    if not header:
        return None
    lines = header.decode('iso-8859-1').split('\r\n')
    request_line = lines[0].split(' ')
    if len(request_line) < 2:
        return None
    headers: Dict[str, str] = {}
    for line in lines[1:]:
        if not line or ':' not in line:
            continue
        key, value = line.split(':', maxsplit=1)
        headers[key.strip().lower()] = value.strip()
    content_length = int(headers.get('content-length', '0') or '0')
    if content_length > 0:
        await reader.readexactly(content_length)
    return request_line[0].upper(), request_line[1], headers


async def _read_until(reader, separator: bytes) -> bytes:
    try:
        return await reader.readuntil(separator)
    except asyncio.IncompleteReadError as error:
        return error.partial


async def _write_rtsp_response(
        writer, cseq, *, status_code=200, status_text='OK', headers=None, body=b'') -> None:
    response_headers = {
        'CSeq': cseq,
        'Server': RTSP_SERVER_NAME,
        'Content-Length': str(len(body)),
        **(headers or {}),
    }
    writer.write(f'RTSP/1.0 {status_code} {status_text}\r\n'.encode())
    for key, value in response_headers.items():
        writer.write(f'{key}: {value}\r\n'.encode())
    writer.write(b'\r\n')
    writer.write(body)
    await writer.drain()


def _content_base(uri: str) -> str:
    return uri if uri.endswith('/') else f'{uri}/'


def _interleaved_channel(transport: str) -> int:
    marker = 'interleaved='
    if marker not in transport:
        return 0
    value = transport.split(marker, maxsplit=1)[1].split(';', maxsplit=1)[0]
    first = value.split('-', maxsplit=1)[0]
    return int(first) if first.isdecimal() else 0
# --- END vendored core -------------------------------------------------------


class LatestFrame:
    """Thread-safe holder for the most recent RTP-ready frame of one camera."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: Optional[JpegRtpFrame] = None

    def set(self, frame: JpegRtpFrame) -> None:
        with self._lock:
            self._frame = frame

    def get(self) -> Optional[JpegRtpFrame]:
        with self._lock:
            return self._frame


class RtspLiveServer:
    """RTSP/TCP server streaming one camera's LATEST frame as RTP/JPEG."""

    def __init__(
        self,
        *,
        name: str,
        source_key: str,
        listen_host: str,
        advertise_host: str,
        port: int,
        fps: float,
        latest: LatestFrame,
    ) -> None:
        if fps <= 0:
            raise ValueError('fps must be positive')
        self._name = name
        self._source_key = source_key
        self._listen_host = listen_host
        self._advertise_host = advertise_host
        self._port = port
        self._fps = float(fps)
        self._latest = latest
        self._server: Optional[asyncio.AbstractServer] = None
        self._client_tasks: set = set()
        self._client_writers: set = set()
        self._ssrc = random.getrandbits(32)
        self._stopping = False

    async def start(self) -> None:
        self._server = await asyncio.start_server(
            self._handle_client, self._listen_host, self._port)

    async def stop(self) -> None:
        self._stopping = True
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        for writer in list(self._client_writers):
            writer.close()
        current = asyncio.current_task()
        tasks = {task for task in self._client_tasks if task is not current and not task.done()}
        for task in tasks:
            task.cancel()
        if tasks:
            _, pending = await asyncio.wait(tasks, timeout=SHUTDOWN_TIMEOUT_SECONDS)
            for task in pending:
                task.cancel()

    async def serve_forever(self) -> None:
        if self._server is None:
            raise RuntimeError('RTSP server has not been started')
        async with self._server:
            await self._server.serve_forever()

    async def _handle_client(self, reader, writer) -> None:
        task = asyncio.current_task()
        if task is not None:
            self._client_tasks.add(task)
        self._client_writers.add(writer)
        client = writer.get_extra_info('peername')
        logger.info('[loop] rtsp %s: client connected %s', self._source_key, client)
        interleaved_channel = 0
        try:
            while not self._stopping and not reader.at_eof():
                request = await _read_rtsp_request(reader)
                if request is None:
                    return
                method, uri, headers = request
                cseq = headers.get('cseq', '1')
                if method == 'OPTIONS':
                    await _write_rtsp_response(
                        writer, cseq,
                        headers={'Public': 'OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN'})
                elif method == 'DESCRIBE':
                    await _write_rtsp_response(
                        writer, cseq,
                        headers={
                            'Content-Base': _content_base(uri),
                            'Content-Type': 'application/sdp',
                        },
                        body=self._sdp().encode(),
                    )
                elif method == 'SETUP':
                    interleaved_channel = _interleaved_channel(headers.get('transport', ''))
                    await _write_rtsp_response(
                        writer, cseq,
                        headers={
                            'Session': RTSP_SESSION_ID,
                            'Transport': (
                                'RTP/AVP/TCP;unicast;'
                                f'interleaved={interleaved_channel}-{interleaved_channel + 1};'
                                f'ssrc={self._ssrc:08X}'
                            ),
                        },
                    )
                elif method == 'PLAY':
                    await _write_rtsp_response(
                        writer, cseq,
                        headers={'Session': RTSP_SESSION_ID, 'Range': 'npt=0.000-'})
                    await self._stream(writer, interleaved_channel)
                    return
                elif method == 'TEARDOWN':
                    await _write_rtsp_response(writer, cseq, headers={'Session': RTSP_SESSION_ID})
                    return
                else:
                    await _write_rtsp_response(
                        writer, cseq, status_code=405, status_text='Method Not Allowed')
        except (ConnectionError, BrokenPipeError, asyncio.IncompleteReadError):
            return
        finally:
            self._client_writers.discard(writer)
            if task is not None:
                self._client_tasks.discard(task)
            writer.close()
            with contextlib.suppress(ConnectionError, BrokenPipeError):
                await writer.wait_closed()
            logger.info('[loop] rtsp %s: client disconnected %s', self._source_key, client)

    def _sdp(self) -> str:
        return '\r\n'.join([
            'v=0',
            f'o=- 0 0 IN IP4 {self._advertise_host}',
            f's={self._name}',
            't=0 0',
            'a=control:*',
            f'm=video 0 RTP/AVP/TCP {RTP_PAYLOAD_TYPE}',
            f'a=rtpmap:{RTP_PAYLOAD_TYPE} JPEG/{RTP_CLOCK_RATE}',
            'a=control:trackID=0',
            '',
        ])

    async def _stream(self, writer, channel: int) -> None:
        sequence = random.randrange(0, 2**16)
        frame_interval = 1.0 / self._fps
        started = time.monotonic()
        next_send = started
        while not self._stopping and not writer.is_closing():
            delay = next_send - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            send_at = time.monotonic()
            frame = self._latest.get()
            if frame is not None:
                timestamp = int(max(0.0, send_at - started) * RTP_CLOCK_RATE) % 2**32
                sequence = await _write_mjpeg_rtp_frame(
                    writer, channel, sequence, timestamp, self._ssrc, frame)
            next_send += frame_interval
            if next_send <= send_at:
                next_send = send_at + frame_interval


class RtspServerThread:
    """Run several :class:`RtspLiveServer` instances on one background asyncio loop."""

    def __init__(self) -> None:
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._servers: list = []

    def start(self, servers, *, timeout_s: float) -> None:
        """Bind every server and start serving; raises if binding fails or times out."""
        ready = threading.Event()
        failure: list = []

        def _run() -> None:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            async def _start_all() -> None:
                for server in servers:
                    await server.start()
                    self._servers.append(server)
                    asyncio.ensure_future(server.serve_forever())

            try:
                self._loop.run_until_complete(_start_all())
            except Exception as error:  # surface bind errors to the caller thread
                failure.append(error)
                ready.set()
                return
            ready.set()
            self._loop.run_forever()

        self._thread = threading.Thread(target=_run, name='loop-rtsp', daemon=True)
        self._thread.start()
        if not ready.wait(timeout=timeout_s):
            raise ConnectionError('[loop] RTSP servers failed to start within timeout')
        if failure:
            raise failure[0]

    def stop(self) -> None:
        loop = self._loop
        if loop is not None:
            async def _stop_all() -> None:
                for server in self._servers:
                    with contextlib.suppress(Exception):
                        await server.stop()
            with contextlib.suppress(Exception):
                future = asyncio.run_coroutine_threadsafe(_stop_all(), loop)
                future.result(timeout=SHUTDOWN_TIMEOUT_SECONDS + 1.0)
            loop.call_soon_threadsafe(loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=SHUTDOWN_TIMEOUT_SECONDS)
        self._servers = []
        self._loop = None
        self._thread = None


DEFAULT_RTSP_BASE_PORT = 8554
DEFAULT_LISTEN_HOST = '0.0.0.0'
LOCAL_HOSTS = frozenset({'', 'localhost', '127.0.0.1', '::1', '0.0.0.0'})


def detect_advertise_host(loop_addr: str) -> str:
    """
    Return the local address Loop should use to reach this host's RTSP servers.

    When Loop runs on the same machine ``127.0.0.1`` is correct. Otherwise the
    outbound interface towards the Loop host is looked up without sending data.
    """
    parsed = urlsplit(f'//{loop_addr}')
    host = parsed.hostname or ''
    port = parsed.port or 50051
    if host in LOCAL_HOSTS:
        return '127.0.0.1'
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect((host, port))
            return probe.getsockname()[0]
    except OSError as error:
        logger.warning('[loop] could not detect the RTSP advertise host towards %s (%s); '
                       'falling back to 127.0.0.1', loop_addr, error)
        return '127.0.0.1'


class _RawSlot:
    """Single-slot mailbox for the latest raw frame; wakes the encoder thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._bgr = None
        self._seq = 0
        self.updated = threading.Event()

    def set(self, jpeg: Optional[bytes], bgr) -> None:
        with self._lock:
            self._jpeg = jpeg
            self._bgr = bgr
            self._seq += 1
        self.updated.set()

    def get(self) -> Tuple[Optional[bytes], object, int]:
        with self._lock:
            return self._jpeg, self._bgr, self._seq


@dataclass
class _CameraChannel:
    key: str
    source_key: str
    name: str
    rtsp_path: str
    port: int
    width: int
    height: int
    fps: float
    raw: _RawSlot = field(default_factory=_RawSlot)
    latest: LatestFrame = field(default_factory=LatestFrame)
    producer: object = None  # loop_sdk.SourceProducer

    def rtsp_uri(self, advertise_host: str) -> str:
        return f'rtsp://{advertise_host}:{self.port}/{self.rtsp_path}'


class LoopCameraStreamer:
    """
    Serve live camera frames to Config Loop over RTSP/RTP-JPEG.

    ``cameras`` maps a camera name (the robot config key, e.g. ``cam_head``) to
    its ``(width, height)``. Lifecycle::

        streamer = LoopCameraStreamer(loop_addr, {'cam_head': (1280, 720)}, fps=30)
        streamer.connect()                     # bind RTSP servers + advertise via gRPC
        streamer.set_jpeg('cam_head', data)    # from each CompressedImage callback
        streamer.close()                       # in finally

    One monocular RTSP source per camera. ``CompressedImage`` JPEG bytes are
    packetized directly (no decode); anything else is re-encoded with OpenCV.
    """

    def __init__(
        self,
        loop_addr: str,
        cameras: Mapping[str, Tuple[int, int]],
        *,
        fps: float = 30.0,
        advertise_host: Optional[str] = None,
        listen_host: str = DEFAULT_LISTEN_HOST,
        base_port: int = DEFAULT_RTSP_BASE_PORT,
        jpeg_quality: int = 80,
        connect_timeout_s: float = 5.0,
        client_id_prefix: str = 'physical-ai',
    ) -> None:
        if fps <= 0:
            raise ValueError('fps must be positive')
        self._loop_addr = loop_addr
        self._fps = float(fps)
        self._advertise_host = advertise_host or detect_advertise_host(loop_addr)
        self._listen_host = listen_host
        self._base_port = int(base_port)
        self._jpeg_quality = int(jpeg_quality)
        self._connect_timeout_s = connect_timeout_s
        self._client_id_prefix = client_id_prefix
        self._channels: Dict[str, _CameraChannel] = self._build_channels(cameras)
        self._rtsp = RtspServerThread()
        self._encoder_threads: list = []
        self._stop_encoders = threading.Event()
        self._connected = False

    @property
    def advertise_host(self) -> str:
        return self._advertise_host

    @property
    def camera_names(self) -> Tuple[str, ...]:
        return tuple(self._channels)

    def rtsp_uris(self) -> Dict[str, str]:
        return {key: ch.rtsp_uri(self._advertise_host) for key, ch in self._channels.items()}

    @staticmethod
    def source_key_for(camera_name: str) -> str:
        """Return the Loop source key of a camera (no slashes, see loop-sdk docs)."""
        return 'camera-' + camera_name.strip('/').replace('/', '-').replace('_', '-')

    def _build_channels(self, cameras: Mapping[str, Tuple[int, int]]) -> Dict[str, _CameraChannel]:
        channels: Dict[str, _CameraChannel] = {}
        port = self._base_port
        for key, (width, height) in cameras.items():
            if width <= 0 or height <= 0:
                raise ValueError(f'camera {key!r} has an invalid resolution {width}x{height}')
            if width > MAX_RTP_JPEG_DIM or height > MAX_RTP_JPEG_DIM:
                logger.warning('[loop] camera %s %dx%d exceeds the RTP/JPEG limit of %dpx; '
                               'skipping', key, width, height, MAX_RTP_JPEG_DIM)
                continue
            channels[key] = _CameraChannel(
                key=key,
                source_key=self.source_key_for(key),
                name=f'{key} camera',
                rtsp_path=key.strip('/'),
                port=port,
                width=int(width),
                height=int(height),
                fps=self._fps,
            )
            port += 1
        return channels

    def connect(self) -> None:
        """Bind the RTSP servers, start the encoders and advertise every camera."""
        from loop_sdk import (
            CameraOpenResult,
            CameraPixelFormat,
            CameraResolution,
            CameraSetting,
            CameraStreamSchema,
            RtspCameraProtocol,
            SourceProducer,
            SourceSchema,
        )

        if not self._channels:
            logger.warning('[loop] no cameras configured; camera streaming disabled')
            return
        if self._connected:
            return

        # 1) start the asyncio RTSP servers on a background thread.
        servers = [
            RtspLiveServer(
                name=ch.name,
                source_key=ch.source_key,
                listen_host=self._listen_host,
                advertise_host=self._advertise_host,
                port=ch.port,
                fps=ch.fps,
                latest=ch.latest,
            )
            for ch in self._channels.values()
        ]
        self._rtsp.start(servers, timeout_s=self._connect_timeout_s)

        # 2) start per-camera encoder threads (JPEG parsing off the ROS thread).
        self._stop_encoders.clear()
        for ch in self._channels.values():
            thread = threading.Thread(
                target=self._encode_loop, args=(ch,), name=f'loop-enc-{ch.key}', daemon=True)
            thread.start()
            self._encoder_threads.append(thread)

        # 3) advertise each camera over gRPC; on_camera_open returns our fixed
        #    setting + rtsp uri.
        def _make_open_callback(setting, uri):
            def _on_open(_requested):
                return CameraOpenResult(applied_setting=setting, rtsp=RtspCameraProtocol(uri=uri))
            return _on_open

        try:
            for ch in self._channels.values():
                uri = ch.rtsp_uri(self._advertise_host)
                setting = CameraSetting(
                    resolution=CameraResolution(width=ch.width, height=ch.height),
                    fps=ch.fps,
                    pixel_format=CameraPixelFormat.MJPEG,
                )
                ch.producer = SourceProducer.connect(
                    loop_addr=self._loop_addr,
                    source=SourceSchema(
                        camera=CameraStreamSchema(
                            source_key=ch.source_key,
                            name=ch.name,
                            rtsp=RtspCameraProtocol(uri=uri),
                            available_settings=(setting,),
                        ),
                    ),
                    client_id=f'{self._client_id_prefix}-{ch.rtsp_path}',
                    on_camera_open=_make_open_callback(setting, uri),
                )
                logger.info('[loop] camera %s advertised to %s, serving %s (%dx%d @ %g fps)',
                            ch.source_key, self._loop_addr, uri, ch.width, ch.height, ch.fps)
        except Exception:
            self.close()
            raise
        self._connected = True

    def set_jpeg(self, camera_name: str, jpeg: bytes) -> None:
        """Hand the latest JPEG bytes (``CompressedImage.data``) of a camera in."""
        ch = self._channels.get(camera_name)
        if ch is not None and jpeg:
            ch.raw.set(bytes(jpeg), None)

    def set_bgr(self, camera_name: str, bgr) -> None:
        """Hand the latest decoded BGR frame of a camera in (re-encoded on a worker)."""
        ch = self._channels.get(camera_name)
        if ch is not None and bgr is not None:
            ch.raw.set(None, bgr)

    def _encode_loop(self, ch: _CameraChannel) -> None:
        """Turn the latest raw frame into an RTP-ready frame, off the ROS thread."""
        cv2 = None
        numpy = None
        try:
            import cv2
            import numpy
        except Exception as error:  # cv2 is only needed for the re-encode fallback
            logger.warning('[loop] cv2 unavailable for %s; JPEG pass-through only: %s',
                           ch.source_key, error)
        last_seq = 0
        warned_fallback = False
        warned_failure = False
        while not self._stop_encoders.is_set():
            if not ch.raw.updated.wait(timeout=0.5):
                continue
            ch.raw.updated.clear()
            jpeg, bgr, seq = ch.raw.get()
            if seq == last_seq or (jpeg is None and bgr is None):
                continue
            last_seq = seq
            frame = None
            # fast path: CompressedImage JPEG bytes are already RTP/JPEG compatible
            if jpeg is not None:
                try:
                    frame = parse_jpeg_for_rtp(jpeg)
                except Exception as error:
                    if not warned_fallback:
                        logger.warning('[loop] %s: image is not RTP/JPEG compatible (%s); '
                                       're-encoding with OpenCV', ch.source_key, error)
                        warned_fallback = True
            # slow path: decode / re-encode via cv2
            if frame is None and cv2 is not None:
                try:
                    image = bgr
                    if image is None and jpeg is not None:
                        image = cv2.imdecode(numpy.frombuffer(jpeg, dtype=numpy.uint8),
                                             cv2.IMREAD_COLOR)
                    if image is not None:
                        ok, encoded = cv2.imencode(
                            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
                        if ok:
                            frame = parse_jpeg_for_rtp(encoded.tobytes())
                except Exception as error:
                    if not warned_failure:
                        logger.warning('[loop] %s: frame encode failed: %s', ch.source_key, error)
                        warned_failure = True
            if frame is not None:
                ch.latest.set(frame)

    def close(self) -> None:
        self._stop_encoders.set()
        for thread in self._encoder_threads:
            thread.join(timeout=2.0)
        self._encoder_threads = []
        for ch in self._channels.values():
            producer = ch.producer
            ch.producer = None
            if producer is not None:
                with contextlib.suppress(Exception):
                    producer.close()
        self._rtsp.stop()
        self._connected = False


# ============================================================================
# Smoke test (no ROS, no robot, no cameras)
# ============================================================================
SMOKE_JOINT_ORDER = {
    'leader_left': [f'arm_l_joint{i}' for i in range(1, 8)] + ['gripper_l_joint1'],
    'leader_right': [f'arm_r_joint{i}' for i in range(1, 8)] + ['gripper_r_joint1'],
    'leader_head': ['head_joint1', 'head_joint2'],
    'leader_lift': ['lift_joint'],
    'leader_mobile': [],
}


def _test_pattern(width: int, height: int, phase: float):
    """Return a moving colour-bar BGR frame so motion / decoding is obvious in the UI."""
    import numpy as np

    image = np.zeros((height, width, 3), dtype=np.uint8)
    bars = 8
    for index in range(bars):
        x0 = (index * width) // bars
        x1 = ((index + 1) * width) // bars
        image[:, x0:x1] = [(index * 32) % 256, (index * 64) % 256, (index * 96) % 256]
    return np.roll(image, int((phase * width) % width), axis=1)


def smoke_test(argv: Optional[Sequence[str]] = None) -> None:
    """
    Stream synthetic joints and a test pattern to a running Config Loop.

    Verifies, before touching the robot, that loop-sdk is installed and Loop is
    reachable (fail-fast), that the ``robot-step`` channels appear in the Loop
    UI, and that the ``camera-cam-head`` source decodes as a real image (NOT
    green noise). ``ffplay rtsp://127.0.0.1:8554/cam_head`` checks the RTSP side
    independently of Loop.
    """
    import argparse
    import math

    parser = argparse.ArgumentParser(description='Config Loop streamer smoke test (no ROS)')
    parser.add_argument('--loop-addr', default=DEFAULT_LOOP_ADDR)
    parser.add_argument('--robot-type', default='smoke_test')
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--rtsp-advertise-host', default=None)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(message)s')

    mapper = RobotStepMapper(args.robot_type, build_groups(SMOKE_JOINT_ORDER))
    robot = LoopRobotStreamer(args.loop_addr, mapper)
    robot.connect()
    print(f'[smoke] robot-step connected: {len(mapper.channel_keys())} channels')
    camera = None
    if not args.no_camera:
        camera = LoopCameraStreamer(
            args.loop_addr, {'cam_head': (args.width, args.height)},
            fps=args.fps, advertise_host=args.rtsp_advertise_host)
        camera.connect()
        print(f'[smoke] camera advertised: {camera.rtsp_uris()}')

    joint_names = [name for group in mapper.groups for name in group.joint_names
                   if not group.mobile]
    period = 1.0 / args.fps
    started = time.monotonic()
    step = 0
    try:
        while time.monotonic() - started < args.seconds:
            tick = time.monotonic()
            phase = step * 0.02
            follower = {name: 0.3 * math.sin(phase + i) for i, name in enumerate(joint_names)}
            leader = {
                group.name: {name: follower[name] + 0.05 for name in group.joint_names}
                for group in mapper.groups if not group.mobile
            }
            mobile = (0.1 * math.sin(phase), 0.0, 0.05 * math.cos(phase))
            robot.send(mapper.build_payload(
                follower_positions=follower, leader_positions=leader,
                mobile_state=mobile, mobile_command=mobile))
            if camera is not None and step % 2 == 0:
                camera.set_bgr('cam_head', _test_pattern(args.width, args.height, phase))
            step += 1
            if step % int(args.fps * 5) == 0:
                print(f'[smoke] {robot.sent_count} ticks sent, stats={robot.stats()}')
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.close()
        robot.close()
        print(f'[smoke] done: {robot.sent_count} robot-step ticks')


if __name__ == '__main__':
    smoke_test()
