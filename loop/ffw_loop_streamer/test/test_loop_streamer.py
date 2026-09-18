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

"""Offline tests for the Config Loop streamer (no ROS, no Loop instance needed)."""

from ffw_loop_streamer.loop_streamer import (
    build_groups,
    group_label,
    jpeg_dimensions,
    LoopCameraStreamer,
    LoopRobotStreamer,
    MAX_RTP_PAYLOAD,
    mjpeg_rtp_payloads,
    parse_jpeg_for_rtp,
    RobotStepMapper,
    rtp_header,
    RTP_PAYLOAD_TYPE,
)
import pytest

FFW_JOINT_ORDER = {
    'joint_order.leader_left': ['arm_l_joint1', 'arm_l_joint2', 'gripper_l_joint1'],
    'joint_order.leader_right': ['arm_r_joint1', 'arm_r_joint2', 'gripper_r_joint1'],
    'joint_order.leader_head': ['head_joint1', 'head_joint2'],
    'joint_order.leader_mobile': ['linear_x', 'linear_y', 'angular_z'],
}


def _mapper() -> RobotStepMapper:
    return RobotStepMapper('ffw_sg2_rev1', build_groups(FFW_JOINT_ORDER))


# --------------------------------------------------------------- state mapping
def test_group_labels_strip_leader_prefix():
    assert group_label('leader_left') == 'left'
    assert group_label('joint_order.leader_head') == 'head'
    assert group_label('leader') == 'arm'
    assert group_label('leader_mobile') == 'mobile'


def test_build_groups_marks_mobile_and_keeps_order():
    groups = build_groups(FFW_JOINT_ORDER)
    assert [group.label for group in groups] == ['left', 'right', 'head', 'mobile']
    assert groups[0].joint_names == ('arm_l_joint1', 'arm_l_joint2', 'gripper_l_joint1')
    assert not groups[0].mobile
    assert groups[3].mobile
    assert groups[3].joint_names == ('linear_x', 'linear_y', 'angular_z')


def test_channel_layout_is_fixed_and_dotted():
    assert _mapper().channel_keys() == [
        'ffw_sg2_rev1.observation.left.joint_position',
        'ffw_sg2_rev1.observation.right.joint_position',
        'ffw_sg2_rev1.observation.head.joint_position',
        'ffw_sg2_rev1.observation.mobile.velocity',
        'ffw_sg2_rev1.action.left.joint_position',
        'ffw_sg2_rev1.action.right.joint_position',
        'ffw_sg2_rev1.action.head.joint_position',
        'ffw_sg2_rev1.action.mobile.velocity',
    ]


def test_payload_orders_follower_by_group_joint_names():
    mapper = _mapper()
    follower = {
        'gripper_l_joint1': 0.9, 'arm_l_joint2': 0.2, 'arm_l_joint1': 0.1,
        'arm_r_joint1': 1.1, 'arm_r_joint2': 1.2, 'gripper_r_joint1': 1.9,
        'head_joint1': 2.1, 'head_joint2': 2.2,
    }
    leader = {
        'leader_left': {'arm_l_joint1': 0.11, 'arm_l_joint2': 0.21, 'gripper_l_joint1': 0.91},
        'leader_head': {'head_joint2': 2.21, 'head_joint1': 2.11},
    }
    payload = mapper.build_payload(
        follower_positions=follower,
        leader_positions=leader,
        mobile_state=(0.5, 0.0, -0.1),
        mobile_command=None,
    )
    assert list(payload) == mapper.channel_keys()
    assert payload['ffw_sg2_rev1.observation.left.joint_position'] == [0.1, 0.2, 0.9]
    assert payload['ffw_sg2_rev1.observation.right.joint_position'] == [1.1, 1.2, 1.9]
    assert payload['ffw_sg2_rev1.observation.mobile.velocity'] == [0.5, 0.0, -0.1]
    assert payload['ffw_sg2_rev1.action.left.joint_position'] == [0.11, 0.21, 0.91]
    assert payload['ffw_sg2_rev1.action.head.joint_position'] == [2.11, 2.21]
    # leader_right never published, mobile command absent -> "no reading", not zeros
    assert payload['ffw_sg2_rev1.action.right.joint_position'] is None
    assert payload['ffw_sg2_rev1.action.mobile.velocity'] is None
    assert all(isinstance(value, float)
               for vector in payload.values() if vector is not None for value in vector)


def test_incomplete_group_is_no_reading_not_partial_vector():
    mapper = _mapper()
    follower = {'arm_l_joint1': 0.1, 'arm_l_joint2': 0.2}  # gripper_l_joint1 missing
    payload = mapper.build_payload(follower_positions=follower)
    assert payload['ffw_sg2_rev1.observation.left.joint_position'] is None
    assert mapper.missing_follower_joints(follower) == [
        'gripper_l_joint1', 'arm_r_joint1', 'arm_r_joint2', 'gripper_r_joint1',
        'head_joint1', 'head_joint2',
    ]


def test_mobile_vector_must_have_three_axes():
    payload = _mapper().build_payload(mobile_state=(0.1, 0.2))
    assert payload['ffw_sg2_rev1.observation.mobile.velocity'] is None


def test_single_group_robot_uses_arm_label():
    mapper = RobotStepMapper('omy_f3m', build_groups({'joint_order.leader': ['joint1', 'joint2']}))
    assert mapper.channel_keys() == [
        'omy_f3m.observation.arm.joint_position',
        'omy_f3m.action.arm.joint_position',
    ]


def test_channel_infos_carry_joint_names_as_labels():
    infos = _mapper().channel_infos()
    assert infos[0].label == 'arm_l_joint1, arm_l_joint2, gripper_l_joint1'
    assert infos[0].unit == 'rad'
    assert infos[0].group == 'observation.left'
    assert infos[3].unit == ''


def test_duplicate_labels_and_empty_groups_are_rejected():
    with pytest.raises(ValueError):
        RobotStepMapper('x', build_groups({'leader_a': ['j1'], 'joint_order.leader_a': ['j2']}))
    with pytest.raises(ValueError):
        build_groups({'leader': ['']})


def test_layout_matches_loop_sdk_flatten_and_channel_specs():
    loop_sdk = pytest.importorskip('loop_sdk')
    mapper = _mapper()
    payload = mapper.build_payload(mobile_state=(0.0, 0.0, 0.0))
    assert list(loop_sdk.flatten_step(payload)) == mapper.channel_keys()
    specs = LoopRobotStreamer('localhost:1', mapper).channel_specs(loop_sdk.ChannelSpec)
    assert [spec.key for spec in specs] == mapper.channel_keys()
    assert specs[0].unit == 'rad' and specs[0].group == 'observation.left'


def test_robot_streamer_fails_fast_when_loop_is_unreachable():
    pytest.importorskip('loop_sdk')
    streamer = LoopRobotStreamer('127.0.0.1:9', _mapper(), connect_timeout_s=0.3)
    with pytest.raises(ConnectionError):
        streamer.connect()
    assert streamer.send({}) is False


# ------------------------------------------------------------------ RTP / JPEG
cv2 = pytest.importorskip('cv2')
np = pytest.importorskip('numpy')


def _jpeg(width: int, height: int, quality: int = 80) -> bytes:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, : width // 2] = (0, 128, 255)
    ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    assert ok
    return encoded.tobytes()


def test_jpeg_dimensions_from_header():
    assert jpeg_dimensions(_jpeg(640, 480)) == (640, 480)
    assert jpeg_dimensions(bytearray(_jpeg(1280, 720))) == (1280, 720)
    with pytest.raises(ValueError):
        jpeg_dimensions(b'\x89PNG\r\n\x1a\n' + b'\x00' * 32)


def test_opencv_jpeg_is_rtp_compatible():
    jpeg = _jpeg(640, 480)
    frame = parse_jpeg_for_rtp(jpeg)
    assert (frame.width_blocks, frame.height_blocks) == (80, 60)
    assert frame.jpeg_type in (0, 1)
    assert len(frame.quantization_tables) == 128
    assert frame.scan_data and frame.scan_data in jpeg


def test_rtp_payloads_carry_tables_once_and_reassemble():
    frame = parse_jpeg_for_rtp(_jpeg(320, 240))
    payloads = mjpeg_rtp_payloads(frame)
    assert all(len(payload) <= MAX_RTP_PAYLOAD for payload in payloads)
    first = payloads[0]
    # main header: type-specific(1) offset(3) type(1) q(1) width(1) height(1)
    assert first[0] == 0 and int.from_bytes(first[1:4], 'big') == 0
    assert first[4] == frame.jpeg_type and first[5] == 255
    assert (first[6], first[7]) == (frame.width_blocks, frame.height_blocks)
    # quantization header only on the first fragment
    assert first[8:10] == b'\x00\x00'
    assert int.from_bytes(first[10:12], 'big') == len(frame.quantization_tables)
    scan = first[12 + len(frame.quantization_tables):]
    offset = len(scan)
    for payload in payloads[1:]:
        assert int.from_bytes(payload[1:4], 'big') == offset
        scan += payload[8:]
        offset += len(payload[8:])
    assert scan == frame.scan_data


def test_rtp_header_marker_and_payload_type():
    header = rtp_header(7, 90_000, 0xDEADBEEF, marker=True)
    assert len(header) == 12
    assert header[0] == 0x80
    assert header[1] == 0x80 | RTP_PAYLOAD_TYPE
    assert int.from_bytes(header[2:4], 'big') == 7
    assert rtp_header(7, 0, 0, marker=False)[1] == RTP_PAYLOAD_TYPE


def test_oversize_frame_is_rejected():
    with pytest.raises(ValueError):
        parse_jpeg_for_rtp(_jpeg(2048, 64))


def test_camera_channels_source_keys_ports_and_uris():
    streamer = LoopCameraStreamer(
        'localhost:50051', {'cam_head': (1280, 720), 'cam_wrist_left': (640, 480)},
        fps=30.0, advertise_host='10.0.0.5', base_port=8554)
    assert streamer.advertise_host == '10.0.0.5'
    assert LoopCameraStreamer.source_key_for('cam_wrist_left') == 'camera-cam-wrist-left'
    assert streamer.rtsp_uris() == {
        'cam_head': 'rtsp://10.0.0.5:8554/cam_head',
        'cam_wrist_left': 'rtsp://10.0.0.5:8555/cam_wrist_left',
    }
    # cameras beyond the RTP/JPEG size limit are skipped, not fatal
    skipped = LoopCameraStreamer('localhost:50051', {'big': (4096, 2160)}, advertise_host='h')
    assert skipped.camera_names == ()
