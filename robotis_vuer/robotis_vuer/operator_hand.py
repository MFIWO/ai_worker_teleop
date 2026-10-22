"""Operator pause wrapper; original SH5 coordinate processing stays unchanged."""

import argparse
import copy
import json
import threading
import time

import numpy as np
import rclpy
from robotis_vuer.arm_tracking_guard import ArmTrackingGuard
from robotis_vuer.vr_publisher_hand_neck import NeckVRTrajectoryPublisher
from robotis_vuer.vr_publisher_sh5 import VRTrajectoryPublisher
from std_msgs.msg import Bool, String


class BufferedPublisher:
    def __init__(self, owner, name, publisher):
        self.owner, self.name, self.publisher = owner, name, publisher

    def publish(self, msg):
        owner = self.owner
        with owner.operator_lock:
            if owner.operator_paused:
                return
            if owner.operator_batch is not None:
                owner.operator_batch.append((self.name, self.publisher, msg))
            elif not owner.operator_resuming:
                self.publisher.publish(msg)


class OperatorPauseMixin:
    """Pause both arm poses and finger inputs without opening the hands."""

    def __init__(self):
        self.operator_lock = threading.RLock()
        self.operator_paused = False
        self.operator_resuming = False
        self.operator_batch = None
        self.operator_guard = ArmTrackingGuard()
        self.operator_hands = {'left': None, 'right': None}
        self.operator_request_id = ''
        self.operator_reason = 'ready'
        super().__init__()
        if any((self.enable_lift_publishing, self.enable_head_publishing,
                self.enable_base_publishing)):
            raise ValueError('Operator console requires VR head/base/lift publishing disabled')
        self.operator_outputs = {}
        for side in ('left', 'right'):
            for suffix in ('wrist_rviz_pub', 'elbow_pub', 'shoulder_pub',
                           'hand_pos_pub', 'publisher_'):
                name = f'{side}_{suffix}'
                publisher = getattr(self, name)
                self.operator_outputs[name] = publisher
                setattr(self, name, BufferedPublisher(self, name, publisher))
        self.operator_status_pub = self.create_publisher(String, '/vr/operator/status', 10)
        self.operator_sub = self.create_subscription(
            String, '/vr/operator/request', self.operator_request, 10)
        self.operator_timer = self.create_timer(.1, self.operator_status)

    def operator_status(self):
        with self.operator_lock:
            self.operator_status_pub.publish(String(data=json.dumps({
                'paused': self.operator_paused, 'resuming': self.operator_resuming,
                'active': bool(self.vr_publishing_enabled),
                'request_id': self.operator_request_id, 'reason': self.operator_reason,
            })))
            # Repeat the disable while held; never repeatedly re-arm the robot.
            if self.operator_paused or self.operator_resuming:
                self.reactivate_pub.publish(Bool(data=False))

    def operator_request(self, msg):
        try:
            request = json.loads(msg.data)
            command, request_id = request['command'], request['id']
            if command not in ('pause', 'resume') or not isinstance(request_id, str):
                return
        except (ValueError, KeyError, TypeError):
            return
        with self.operator_lock:
            if request_id == self.operator_request_id:
                return
            self.operator_request_id = request_id
            if command == 'pause':
                self.operator_paused = True
                self.operator_resuming = False
                self.operator_reason = 'paused'
                self.operator_guard.hold('operator_pause')
                self.reactivate_pub.publish(Bool(data=False))
            elif self.operator_paused:
                self.operator_paused = False
                self.operator_resuming = bool(self.vr_publishing_enabled)
                self.operator_reason = (
                    'waiting_for_fresh_nearby_hands' if self.operator_resuming
                    else 'gesture_activation_required')
            self.operator_status()

    def publish_zero_hand_joint_trajectories(self):
        # Operator mode keeps the last finger goal when inactive; disabling must not open hands.
        return

    def publish_relative_pose(
        self, camera_relative_position, camera_relative_quaternion, publisher,
        vr_scale=1.0, x_offset=0.0, y_offset=0.0, z_offset=0.0,
        apply_right_z_flip=False, pose_role='wrist', side='', stamp=None,
    ):
        if (self.operator_batch is not None
                and not isinstance(self, NeckVRTrajectoryPublisher)
                and pose_role in ('wrist', 'elbow', 'shoulder')):
            # Compare pre-filter poses too: smoothing must not conceal a reacquisition jump.
            self._operator_raw_poses[f'{side}_{pose_role}_raw'] = (
                np.array(camera_relative_position, dtype=float) * vr_scale,
                np.array(camera_relative_quaternion, dtype=float))
        return super().publish_relative_pose(
            camera_relative_position, camera_relative_quaternion, publisher,
            vr_scale=vr_scale, x_offset=x_offset, y_offset=y_offset, z_offset=z_offset,
            apply_right_z_flip=apply_right_z_flip, pose_role=pose_role, side=side, stamp=stamp)

    def _update_gesture_toggle(self, left, right):
        with self.operator_lock:
            if self.operator_paused or self.operator_resuming:
                self.gesture_combo_hold_start_time = None
                self.gesture_toggle_latched = False
                self.gesture_combo_active_prev = False
                return
            return super()._update_gesture_toggle(left, right)

    def reactivate_callback(self, msg):
        with self.operator_lock:
            if (self.operator_paused or self.operator_resuming) and msg.data:
                return
            return super().reactivate_callback(msg)

    async def on_hand_move(self, event, session):
        with self.operator_lock:
            if isinstance(event.value, dict):
                for side in ('left', 'right'):
                    if side not in event.value:
                        continue
                    try:
                        data = np.asarray(event.value[side], dtype=float)
                        valid = data.size == 400 and np.all(np.isfinite(data))
                        wrist = data[:16].reshape(4, 4, order='F') if valid else None
                        valid = (valid and np.allclose(wrist[3], [0, 0, 0, 1], atol=.001)
                                 and np.allclose(wrist[:3, :3].T @ wrist[:3, :3],
                                                 np.eye(3), atol=.01)
                                 and np.isclose(np.linalg.det(wrist[:3, :3]), 1., atol=.01))
                    except (TypeError, ValueError):
                        valid = False
                    self.operator_hands[side] = time.monotonic() if valid else None
            await super().on_hand_move(event, session)

    async def on_body_tracking_move(self, event, session):
        with self.operator_lock:
            if self.operator_paused:
                return  # No zero-finger commands, filter updates or arm reference updates.
            now = time.monotonic()
            if self.operator_resuming and any(
                stamp is None or now - stamp > .25
                for stamp in self.operator_hands.values()
            ):
                self.operator_reason = 'waiting_for_fresh_hands'
                return
            previous_filters = copy.deepcopy(self.pose_filters)
            self.operator_batch = []
            self._operator_raw_poses = {}
            try:
                await super().on_body_tracking_move(event, session)
                poses = {}
                for name, _, msg in self.operator_batch:
                    if hasattr(msg, 'pose'):
                        p, q = msg.pose.position, msg.pose.orientation
                        poses[name] = (np.array([p.x, p.y, p.z]),
                                       np.array([q.x, q.y, q.z, q.w]))
                complete = len(poses) == 6
                poses.update(self._operator_raw_poses)
                if self.operator_resuming:
                    if not complete or not self.operator_guard.accept(poses, now):
                        self.pose_filters = previous_filters
                        self.operator_reason = (self.operator_guard.reason if poses
                                                else 'waiting_for_complete_arm_frame')
                        return
                    self.operator_resuming = False
                    self.operator_reason = 'resumed'
                    # Publish coherent goals first; re-arm with existing startup checks.
                    for _, publisher, msg in self.operator_batch:
                        publisher.publish(msg)
                    self.reactivate_pub.publish(Bool(data=True))
                else:
                    for _, publisher, msg in self.operator_batch:
                        publisher.publish(msg)
                if complete:
                    self.operator_guard.last = poses
            finally:
                self.operator_batch = None


class ConsoleNeckPublisher(OperatorPauseMixin, NeckVRTrajectoryPublisher):
    pass


class ConsoleOriginalPublisher(OperatorPauseMixin, VRTrajectoryPublisher):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('neck', 'original'), default='neck')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    cls = ConsoleNeckPublisher if args.mode == 'neck' else ConsoleOriginalPublisher
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reactivate_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
