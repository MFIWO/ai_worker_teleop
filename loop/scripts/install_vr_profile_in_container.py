"""Install the additive VR Loop profile inside the robot container; no nodes started."""
import argparse
from datetime import datetime, timezone
from pathlib import Path

PROFILE = '# VR actions are final commanded joint angles, not wrist poses or measured state.\n# Keep arm and gripper channels separate: a missing gripper must not erase the arm.\n# The original ffw_sg2_rev1 leader profile remains unchanged.\n/**:\n  ros__parameters:\n    ffw_sg2_vr:\n      observation_list: [cam_head, cam_wrist_left, cam_wrist_right, state]\n      camera_topic_list:\n        - cam_head:/zed/zed_node/left/image_rect_color/compressed\n        - cam_wrist_left:/camera_left/camera_left/color/image_rect_raw/compressed\n        - cam_wrist_right:/camera_right/camera_right/color/image_rect_raw/compressed\n      joint_topic_list:\n        - follower_upper_body:/joint_states\n        - leader_left_arm:/leader/joint_trajectory_command_broadcaster_left/joint_trajectory\n        - leader_right_arm:/leader/joint_trajectory_command_broadcaster_right/joint_trajectory\n        - leader_left_gripper:/leader/joint_trajectory_command_broadcaster_left/joint_trajectory\n        - leader_right_gripper:/leader/joint_trajectory_command_broadcaster_right/joint_trajectory\n      joint_list:\n        - leader_left_arm\n        - leader_right_arm\n        - leader_left_gripper\n        - leader_right_gripper\n      joint_order:\n        leader_left_arm:\n          - arm_l_joint1\n          - arm_l_joint2\n          - arm_l_joint3\n          - arm_l_joint4\n          - arm_l_joint5\n          - arm_l_joint6\n          - arm_l_joint7\n        leader_right_arm:\n          - arm_r_joint1\n          - arm_r_joint2\n          - arm_r_joint3\n          - arm_r_joint4\n          - arm_r_joint5\n          - arm_r_joint6\n          - arm_r_joint7\n        leader_left_gripper: [gripper_l_joint1]\n        leader_right_gripper: [gripper_r_joint1]\n'

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--package-root', default='/root/ros2_ws/src/ai_worker/ffw_loop_streamer')
args = parser.parse_args()
root = Path(args.package_root)
launch = root / 'launch/loop_streamer.launch.py'
config = root / 'config/ffw_sg2_vr.yaml'
source = launch.read_text()
updated = source
parameter = "            'source_key': LaunchConfiguration('source_key'),\n"
argument = "        DeclareLaunchArgument(\n            'source_key', default_value='robotis',\n            description='Loop source identity; use robotis-vr for the VR profile'),\n"
if parameter not in updated:
    anchor = "            'fps': LaunchConfiguration('fps'),\n"
    if updated.count(anchor) != 1:
        raise SystemExit('Unexpected launch parameters; no changes made')
    updated = updated.replace(anchor, anchor + parameter)
if "'source_key', default_value=" not in updated:
    anchor = "        DeclareLaunchArgument(\n            'fps', default_value='30.0',"
    if updated.count(anchor) != 1:
        raise SystemExit('Unexpected launch arguments; no changes made')
    updated = updated.replace(anchor, argument + anchor)
compile(updated, str(launch), 'exec')
if config.exists() and config.read_text() != PROFILE:
    raise SystemExit('Existing VR profile differs; no changes made')
if source != updated:
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    backup = launch.with_name(launch.name + '.before_vr_' + stamp + '.txt')
    backup.write_text(source)
    launch.write_text(updated)
config.write_text(PROFILE)
print('VR profile installed. Original ffw_sg2_rev1 profile and default source robotis retained.')
print('Rebuild ffw_loop_streamer before using robot_type:=ffw_sg2_vr source_key:=robotis-vr.')
