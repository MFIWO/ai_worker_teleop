"""Validate live tracking on preview outputs only, while old VR is still running.

This subscribes to the direct-remapped /[lr]_goal_pose inputs. Restart the normal
VR publisher without those remaps after this check, before using the new launch.
"""

from collections import Counter
import json
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scripts.arm_retargeting import ArmRetargetingTeleop


rclpy.init(args=[
    '--ros-args',
    '-p', 'r_wrist_pose_topic:=/r_goal_pose',
    '-p', 'l_wrist_pose_topic:=/l_goal_pose',
    '-p', 'r_goal_pose_topic:=/quest_preview/r_goal_pose',
    '-p', 'l_goal_pose_topic:=/quest_preview/l_goal_pose',
    '-p', 'r_subgoal_pose_topic:=/quest_preview/r_subgoal_pose',
    '-p', 'l_subgoal_pose_topic:=/quest_preview/l_subgoal_pose',
])
arm = ArmRetargetingTeleop()
for publisher in [arm.right_goal_publisher_, arm.left_goal_publisher_,
                  arm.right_subgoal_publisher_, arm.left_subgoal_publisher_]:
    assert publisher.topic_name.startswith('/quest_preview/'), publisher.topic_name

observer = Node('quest_arm_preview_observer')
counts = Counter()
actual = {}
last = {}


def position(msg):
    p = msg.pose.position
    return np.array([p.x, p.y, p.z])


def current(side, msg):
    actual[side] = position(msg)


def preview(side, msg):
    counts[side] += 1
    value = position(msg)
    record = {'xyz': value.tolist(), 'frame': msg.header.frame_id}
    if side in actual:
        record['actual_xyz'] = actual[side].tolist()
        record['position_error_m'] = float(np.linalg.norm(value - actual[side]))
    last[side] = record


for side in ('l', 'r'):
    observer.create_subscription(
        PoseStamped, f'/{side}_gripper_pose',
        lambda msg, side=side: current(side, msg), qos_profile_sensor_data)
    observer.create_subscription(
        PoseStamped, f'/quest_preview/{side}_goal_pose',
        lambda msg, side=side: preview(side, msg), qos_profile_sensor_data)

executor = SingleThreadedExecutor()
executor.add_node(arm)
executor.add_node(observer)
print('PREVIEW READY: all output topics use /quest_preview/; hold both Grips.', flush=True)
started = time.monotonic()
try:
    while time.monotonic() - started < 40:
        executor.spin_once(timeout_sec=.1)
finally:
    result = {'preview_counts': dict(counts), 'latest': last,
              'robot_commands_published': False}
    Path('/workspace/quest_sg2_teleop/preview_result.json').write_text(
        json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    executor.shutdown()
    observer.destroy_node()
    arm.destroy_node()
    rclpy.shutdown()
