#!/usr/bin/env bash
# Gravity-aligned hand teleop. Run before starting the robot VR controller.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PYTHONPATH="$script_dir/robotis_vuer:${PYTHONPATH:-}"
echo 'Neck mode: preparing SG2 elbows at about 90 degrees.'
echo 'Keep follower running; start the robot VR controller only AFTER READY.'
python3 "$script_dir/robotis_vuer/robotis_vuer/prepare_sg2_ready_pose.py" --duration 5.0
exec python3 "$script_dir/robotis_vuer/robotis_vuer/operator_hand.py" --mode neck --ros-args \
  -r /l_wrist_pose:=/l_goal_pose \
  -r /r_wrist_pose:=/r_goal_pose
