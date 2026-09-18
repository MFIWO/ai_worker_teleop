"""Opt-in gravity-aligned arms using the unchanged original SH5 hand publisher."""

import copy
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from robotis_vuer.arm_tracking_guard import ArmTrackingGuard
from robotis_vuer.gravity_arm_frame import GravityArmFrame
from robotis_vuer.vr_publisher_sh5 import VRTrajectoryPublisher
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String


class NeckVRTrajectoryPublisher(VRTrajectoryPublisher):
    def __init__(self):
        self.arm_frame = GravityArmFrame(self.BODY_HEAD_TO_ROS_POSITION)
        self.tracking_guard = ArmTrackingGuard()
        self.hand_received = {'left': None, 'right': None}
        self.body_received = None
        self._pending_arm = None
        self._tracking_status = None
        super().__init__()
        if not self.hand_pose_is_head_relative:
            raise ValueError('Neck mode requires hand_pose_is_head_relative:=true')
        self.head_diagnostic_pub = self.create_publisher(
            PoseStamped, '/vr/diagnostics/head_pose', self.vr_stream_qos)
        self.reference_diagnostic_pub = self.create_publisher(
            PoseStamped, '/vr/diagnostics/arm_reference_pose', self.vr_stream_qos)
        self.tracking_status_pub = self.create_publisher(
            String, '/vr/diagnostics/arm_tracking_status', 10)
        self.tracking_timer = self.create_timer(.05, self.check_tracking)
        self.get_logger().info(
            'NECK MODE: arm position/orientation use gravity-up and horizontal head-forward. '
            'Original offsets, scale, gestures and joint limits remain in effect.')

    def report_tracking(self):
        status = self.tracking_guard.reason
        if hasattr(self, 'tracking_status_pub'):
            self.tracking_status_pub.publish(String(data=status))
        if status != self._tracking_status:
            self._tracking_status = status
            self.get_logger().info(
                f'[ARM TRACKING] {status}. '
                + ('Following hands.' if status == 'tracking' else
                   'Holding last arm targets; bring hands back near the held pose.'))

    def check_tracking(self):
        now = time.monotonic()
        if self.is_vr_publishing_active():
            times = [self.body_received, *self.hand_received.values()]
            if any(t is None or now - t > .25 for t in times):
                self.tracking_guard.hold('tracking_missing')
            self.report_tracking()

    async def on_hand_move(self, event, session):
        if not isinstance(event.value, dict):
            return
        value = dict(event.value)
        for side in ('left', 'right'):
            if side not in value:
                continue  # Partial events are allowed, but each side must remain fresh.
            try:
                data = np.asarray(value[side], dtype=float)
                valid = data.size == 400 and np.all(np.isfinite(data))
                wrist = data[:16].reshape(4, 4, order='F') if valid else None
                valid = valid and np.allclose(wrist[3], [0., 0., 0., 1.], atol=1e-3)
                if valid:
                    rotation = wrist[:3, :3]
                    valid = (np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-2)
                             and np.isclose(np.linalg.det(rotation), 1., atol=1e-2))
            except (TypeError, ValueError):
                valid = False
            if valid:
                self.hand_received[side] = time.monotonic()
            else:
                self.hand_received[side] = None
                setattr(self, side + '_hand_data', None)
                value.pop(side, None)
                value.pop(side + 'State', None)
                self.tracking_guard.hold('tracking_invalid')
        await super().on_hand_move(SimpleNamespace(value=value), session)

    async def on_body_tracking_move(self, event, session):
        if not isinstance(event.value, dict):
            return
        body = event.value.get('body')
        if body is None:
            return
        try:
            head = self.get_body_joint_matrix_from_flat(body, 'head')
        except (TypeError, ValueError):
            self.arm_frame.correction = None
            self.tracking_guard.hold('head_invalid')
            return
        if head is None or not self.arm_frame.update(head):
            # Do not apply an old reference to a new body sample.
            self.arm_frame.correction = None
            self.tracking_guard.hold('head_invalid')
            return
        self.body_received = time.monotonic()
        if hasattr(self, 'head_diagnostic_pub') and rclpy.ok():
            stamp = self.get_clock().now().to_msg()
            for matrix, publisher in (
                (head, self.head_diagnostic_pub),
                (self.arm_frame.reference, self.reference_diagnostic_pub),
            ):
                position, quat = self.matrix_to_pose(matrix)
                msg = PoseStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = 'xr_world_y_up'
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position.tolist()
                (msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w) = quat.tolist()
                publisher.publish(msg)
        if not self.is_vr_publishing_active():
            self.tracking_guard.hold('teleop_disabled')
            await super().on_body_tracking_move(event, session)
            return
        now = time.monotonic()
        if any(t is None or now - t > .25 for t in self.hand_received.values()):
            self.tracking_guard.hold('tracking_missing')
            self.report_tracking()
            return
        previous_filters = copy.deepcopy(self.pose_filters)
        self._pending_arm = []
        self._raw_arm_poses = {}
        accepted = False
        try:
            await super().on_body_tracking_move(event, session)
            poses = dict(self._raw_arm_poses)
            for publisher, msg, key in self._pending_arm:
                p, q = msg.pose.position, msg.pose.orientation
                poses[key + '_goal'] = (np.array([p.x, p.y, p.z]),
                                        np.array([q.x, q.y, q.z, q.w]))
            if len(self._pending_arm) != 6:
                self.tracking_guard.hold('incomplete_arm_frame')
            else:
                accepted = self.tracking_guard.accept(poses, now)
            if accepted:
                for publisher, msg, key in self._pending_arm:
                    publisher.publish(msg)
        finally:
            if not accepted:
                # A rejected sample must not drag the filter toward a tracking outlier.
                self.pose_filters = previous_filters
            self._pending_arm = None
        self.report_tracking()

    def publish_relative_pose(
        self, camera_relative_position, camera_relative_quaternion, publisher,
        vr_scale=1.0, x_offset=0.0, y_offset=0.0, z_offset=0.0,
        apply_right_z_flip=False, pose_role='wrist', side='', stamp=None,
    ):
        if pose_role in ('wrist', 'elbow', 'shoulder'):
            if self.arm_frame.correction is None:
                return
            camera_relative_position, camera_relative_quaternion = self.arm_frame.transform(
                camera_relative_position, camera_relative_quaternion)
            if getattr(self, '_pending_arm', None) is not None:
                key = f'{side}_{pose_role}'
                rotation = R.from_quat(camera_relative_quaternion)
                if apply_right_z_flip:
                    rotation = rotation * R.from_euler('z', 180., degrees=True)
                self._raw_arm_poses[key + '_raw'] = (
                    camera_relative_position * vr_scale - self.zedm_to_base_offset
                    + np.array([x_offset, y_offset, z_offset]), rotation.as_quat())
                target_publisher = publisher
                publisher = SimpleNamespace(publish=lambda msg: self._pending_arm.append(
                    (target_publisher, msg, key)))
        return super().publish_relative_pose(
            camera_relative_position, camera_relative_quaternion, publisher,
            vr_scale=vr_scale, x_offset=x_offset, y_offset=y_offset, z_offset=z_offset,
            apply_right_z_flip=apply_right_z_flip, pose_role=pose_role, side=side, stamp=stamp)


def main(args=None):
    rclpy.init(args=args)
    node = NeckVRTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
