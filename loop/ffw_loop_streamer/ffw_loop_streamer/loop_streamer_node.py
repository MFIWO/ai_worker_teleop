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
Config Loop sidecar for the ROBOTIS AI Worker (ROS 2 entry point, ffw_loop_streamer).

Read-only telemetry tap, modeled on the GR00T-WholeBodyControl sidecar: reads
the per-robot config YAML in ffw_loop_streamer/config (``camera_topic_list``,
``joint_topic_list``, ``joint_list``, ``joint_order``), subscribes to the camera /
follower / leader topics with the same message-type conventions as
``Communicator`` and forwards both into Config Loop via loop-sdk 0.4.1:

  * follower state + leader action -> ``robot-step`` source over gRPC at ``fps``, and
  * each camera -> an RTSP/RTP-JPEG source that Loop pulls.

Run it alongside the teleop / recording stack::

    ros2 launch ffw_loop_streamer loop_streamer.launch.py robot_type:=ffw_sg2_rev1
        loop_addr:=localhost:50051

The robot can be driven by anything (leader arms, joystick, policy): the sidecar
never publishes to the robot and is independent of the recording state machine.
"""

from functools import partial
import os
import time
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ffw_loop_streamer.loop_streamer import (
    build_groups,
    DEFAULT_LOOP_ADDR,
    DEFAULT_RTSP_BASE_PORT,
    DEFAULT_SOURCE_KEY,
    jpeg_dimensions,
    LoopCameraStreamer,
    LoopRobotStreamer,
    RobotStepMapper,
)
from ffw_loop_streamer.param_utils import (
    declare_parameters,
    load_parameters,
    parse_topic_list_with_names,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory


class LoopStreamerNode(Node):
    """Stream robot state, action and cameras to Config Loop."""

    ROBOT_CONFIG_PARAMS = ['camera_topic_list', 'joint_topic_list', 'joint_list']
    PROBE_PERIOD_S = 0.2
    LOG_THROTTLE_S = 5.0

    def __init__(self):
        super().__init__('loop_streamer')

        self.declare_parameter('robot_type', '')
        self.declare_parameter('loop_addr', os.environ.get('LOOP_ADDR', DEFAULT_LOOP_ADDR))
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('source_key', DEFAULT_SOURCE_KEY)
        self.declare_parameter('enable_camera', True)
        self.declare_parameter('rtsp_advertise_host', '')
        self.declare_parameter('rtsp_base_port', DEFAULT_RTSP_BASE_PORT)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('camera_probe_timeout_s', 10.0)
        self.declare_parameter('connect_timeout_s', 5.0)

        self.robot_type: str = self.get_parameter('robot_type').value
        if not self.robot_type:
            raise ValueError("parameter 'robot_type' is required (e.g. ffw_sg2_rev1)")
        self.loop_addr: str = self.get_parameter('loop_addr').value
        self.fps: float = float(self.get_parameter('fps').value)
        if self.fps <= 0:
            raise ValueError("parameter 'fps' must be positive")
        self.enable_camera: bool = bool(self.get_parameter('enable_camera').value)
        self.camera_probe_timeout_s = float(self.get_parameter('camera_probe_timeout_s').value)
        self.connect_timeout_s = float(self.get_parameter('connect_timeout_s').value)

        self._load_robot_config()

        self._camera_msgs: Dict[str, Optional[CompressedImage]] = {}
        self._follower_joint_msgs: Dict[str, Optional[JointState]] = {}
        self._follower_mobile_msg: Optional[Odometry] = None
        self._leader_joint_msgs: Dict[str, Optional[JointTrajectory]] = {}
        self._leader_mobile_msg: Optional[Twist] = None
        self._warned_missing_joints = False

        self._qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._init_subscribers()

        # Robot state/action -> Config Loop (fail-fast if Loop is unreachable).
        self.robot_streamer = LoopRobotStreamer(
            self.loop_addr,
            self.mapper,
            source_key=self.get_parameter('source_key').value,
            connect_timeout_s=self.connect_timeout_s,
        )
        self.robot_streamer.connect()
        self.get_logger().info(
            f'robot-step source connected to Config Loop at {self.loop_addr} '
            f'({len(self.mapper.channel_keys())} channels, {self.fps:g} Hz)')

        # Cameras -> Config Loop (optional; degrade gracefully if none produce frames).
        self.camera_streamer: Optional[LoopCameraStreamer] = None
        self._probe_started_at = time.monotonic()
        self._probe_timer = None
        if self.enable_camera and self.camera_topics:
            self._probe_timer = self.create_timer(self.PROBE_PERIOD_S, self._probe_cameras)
        elif self.enable_camera:
            self.get_logger().warning('no camera topics in the robot config; camera streaming off')

        self._stream_timer = self.create_timer(1.0 / self.fps, self._stream_tick)
        self._tick_count = 0

    # ------------------------------------------------------------------ setup
    def _load_robot_config(self) -> None:
        declare_parameters(self, self.robot_type, self.ROBOT_CONFIG_PARAMS, default_value=[''])
        params = load_parameters(self, self.robot_type, self.ROBOT_CONFIG_PARAMS)
        joint_order_params = [f'joint_order.{name}' for name in params['joint_list'] if name]
        if not joint_order_params:
            raise ValueError(
                f"robot config for '{self.robot_type}' has no joint_list; is the config YAML "
                'passed as node parameters?')
        declare_parameters(self, self.robot_type, joint_order_params, default_value=[''])
        joint_order = load_parameters(self, self.robot_type, joint_order_params)

        self.camera_topics = parse_topic_list_with_names(params['camera_topic_list'])
        self.joint_topics = parse_topic_list_with_names(params['joint_topic_list'])
        self.mapper = RobotStepMapper(self.robot_type, build_groups(joint_order))

        self.get_logger().info(f'robot type: {self.robot_type}')
        self.get_logger().info(f'camera topics: {self.camera_topics}')
        self.get_logger().info(f'joint topics: {self.joint_topics}')
        self.get_logger().info(f'channels: {self.mapper.channel_keys()}')

    def _init_subscribers(self) -> None:
        for name, topic in self.camera_topics.items():
            self._camera_msgs[name] = None
            self.create_subscription(
                CompressedImage, topic, partial(self._camera_callback, name), self._qos)
            self.get_logger().info(f'subscribed camera {name} -> {topic}')

        for name, topic in self.joint_topics.items():
            lowered = name.lower()
            if 'follower' in lowered:
                if 'mobile' in lowered:
                    self.create_subscription(
                        Odometry, topic, self._follower_mobile_callback, self._qos)
                else:
                    self._follower_joint_msgs[name] = None
                    self.create_subscription(
                        JointState, topic, partial(self._follower_joint_callback, name),
                        self._qos)
            elif 'leader' in lowered:
                if 'mobile' in lowered:
                    self.create_subscription(
                        Twist, topic, self._leader_mobile_callback, self._qos)
                else:
                    self._leader_joint_msgs[name] = None
                    self.create_subscription(
                        JointTrajectory, topic, partial(self._leader_joint_callback, name),
                        self._qos)
            else:
                self.get_logger().error(
                    f"joint topic '{name}' must include 'follower' or 'leader'; skipping")
                continue
            self.get_logger().info(f'subscribed {name} -> {topic}')

    # -------------------------------------------------------------- callbacks
    def _camera_callback(self, name: str, msg: CompressedImage) -> None:
        if name in ('cam_wrist_left', 'cam_wrist_right'):
            # Rotate before caching: Loop probes the first frame for its size.
            try:
                import cv2
                import numpy as np

                image = cv2.imdecode(
                    np.frombuffer(bytes(msg.data), dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if image is None:
                    raise ValueError('could not decode wrist camera image')
                image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                ok, encoded = cv2.imencode(
                    '.jpg', image,
                    [cv2.IMWRITE_JPEG_QUALITY, int(self.get_parameter('jpeg_quality').value)],
                )
                if not ok:
                    raise ValueError('could not encode rotated wrist camera image')
                rotated_msg = CompressedImage()
                rotated_msg.header = msg.header
                rotated_msg.format = 'jpeg'
                rotated_msg.data = encoded.tobytes()
                msg = rotated_msg
            except Exception as error:
                self.get_logger().warning(
                    f'skipping wrist camera frame {name}: {error}',
                    throttle_duration_sec=self.LOG_THROTTLE_S,
                )
                return
        self._camera_msgs[name] = msg
        if self.camera_streamer is not None:
            self.camera_streamer.set_jpeg(name, msg.data)

    def _follower_joint_callback(self, name: str, msg: JointState) -> None:
        self._follower_joint_msgs[name] = msg

    def _follower_mobile_callback(self, msg: Odometry) -> None:
        self._follower_mobile_msg = msg

    def _leader_joint_callback(self, name: str, msg: JointTrajectory) -> None:
        self._leader_joint_msgs[name] = msg

    def _leader_mobile_callback(self, msg: Twist) -> None:
        self._leader_mobile_msg = msg

    # ---------------------------------------------------------------- cameras
    def _probe_cameras(self) -> None:
        """Wait for the first frame of each camera (to learn its size), then advertise."""
        sizes: Dict[str, Tuple[int, int]] = {}
        pending = []
        for name, msg in self._camera_msgs.items():
            if msg is None:
                pending.append(name)
                continue
            size = self._frame_size(msg)
            if size is None:
                pending.append(name)
            else:
                sizes[name] = size

        elapsed = time.monotonic() - self._probe_started_at
        if pending and elapsed < self.camera_probe_timeout_s:
            self.get_logger().info(
                f'waiting for camera frames: {pending}',
                throttle_duration_sec=self.LOG_THROTTLE_S)
            return

        self._probe_timer.cancel()
        self._probe_timer = None
        if pending:
            self.get_logger().warning(
                f'no frames from {pending} within {self.camera_probe_timeout_s:g}s; '
                'streaming without them')
        if not sizes:
            self.get_logger().error('no camera produced a frame; camera streaming disabled')
            return

        streamer = LoopCameraStreamer(
            self.loop_addr,
            sizes,
            fps=self.fps,
            advertise_host=self.get_parameter('rtsp_advertise_host').value or None,
            base_port=int(self.get_parameter('rtsp_base_port').value),
            jpeg_quality=int(self.get_parameter('jpeg_quality').value),
            connect_timeout_s=self.connect_timeout_s,
            client_id_prefix=f'physical-ai-{self.robot_type}',
        )
        try:
            streamer.connect()
        except Exception as error:
            self.get_logger().error(f'camera streaming disabled: {error}')
            streamer.close()
            return
        for name, uri in streamer.rtsp_uris().items():
            self.get_logger().info(f"camera '{name}' {sizes[name][0]}x{sizes[name][1]} -> {uri}")
        # Seed each camera with the frame already held, then follow the callbacks.
        for name in streamer.camera_names:
            msg = self._camera_msgs.get(name)
            if msg is not None:
                streamer.set_jpeg(name, msg.data)
        self.camera_streamer = streamer

    def _frame_size(self, msg: CompressedImage) -> Optional[Tuple[int, int]]:
        try:
            return jpeg_dimensions(msg.data)
        except Exception as error:
            self.get_logger().warning(
                f'camera frame is not a JPEG ({msg.format!r}: {error}); decoding with OpenCV',
                throttle_duration_sec=self.LOG_THROTTLE_S)
        try:
            import cv2
            import numpy as np
            image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            return None
        if image is None:
            return None
        height, width = image.shape[:2]
        return int(width), int(height)

    # ------------------------------------------------------------- robot step
    def _stream_tick(self) -> None:
        follower_positions = self._collect_follower_positions()
        if follower_positions is None:
            self.get_logger().info(
                'waiting for follower joint states...', throttle_duration_sec=self.LOG_THROTTLE_S)
            return
        if not self._warned_missing_joints:
            missing = self.mapper.missing_follower_joints(follower_positions)
            if missing:
                self.get_logger().warning(
                    f'follower does not report {missing}; their groups are sent as "no reading"')
                self._warned_missing_joints = True

        payload = self.mapper.build_payload(
            follower_positions=follower_positions,
            leader_positions=self._collect_leader_positions(),
            mobile_state=self._odometry_vector(self._follower_mobile_msg),
            mobile_command=self._twist_vector(self._leader_mobile_msg),
        )
        self.robot_streamer.send(payload)
        self._tick_count += 1
        if self._tick_count % int(max(self.fps, 1.0) * 30) == 0:
            self.get_logger().info(
                f'streamed {self.robot_streamer.sent_count} robot-step ticks to {self.loop_addr}')

    def _collect_follower_positions(self) -> Optional[Dict[str, float]]:
        if not self._follower_joint_msgs:
            return None
        positions: Dict[str, float] = {}
        for msg in self._follower_joint_msgs.values():
            if msg is None:
                return None
            positions.update(zip(msg.name, msg.position))
        return positions

    def _collect_leader_positions(self) -> Dict[str, Dict[str, float]]:
        leader: Dict[str, Dict[str, float]] = {}
        for name, msg in self._leader_joint_msgs.items():
            if msg is None or not msg.points:
                continue
            leader[name] = dict(zip(msg.joint_names, msg.points[0].positions))
        return leader

    @staticmethod
    def _odometry_vector(msg: Optional[Odometry]) -> Optional[Tuple[float, float, float]]:
        if msg is None:
            return None
        twist = msg.twist.twist
        return (twist.linear.x, twist.linear.y, twist.angular.z)

    @staticmethod
    def _twist_vector(msg: Optional[Twist]) -> Optional[Tuple[float, float, float]]:
        if msg is None:
            return None
        return (msg.linear.x, msg.linear.y, msg.angular.z)

    # --------------------------------------------------------------- teardown
    def close(self) -> None:
        self.get_logger().info(
            f'shutting down (streamed {self.robot_streamer.sent_count} robot-step ticks)')
        if self.camera_streamer is not None:
            self.camera_streamer.close()
            self.camera_streamer = None
        self.robot_streamer.close()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LoopStreamerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
