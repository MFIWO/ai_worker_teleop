#!/usr/bin/env bash
# Hand-only comparison: unmodified SH5 publisher with direct wrist goals.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PYTHONPATH="$script_dir/robotis_vuer:${PYTHONPATH:-}"
echo 'Preparing xr_tele SG2 ready pose (elbows -1.57 rad, about 90 degrees).'
echo 'Keep follower bringup running. Start the robot VR controller AFTER READY.'
python3 "$script_dir/robotis_vuer/robotis_vuer/prepare_sg2_ready_pose.py" --duration 5.0
exec python3 "$script_dir/robotis_vuer/robotis_vuer/operator_hand.py" --mode original --ros-args \
  -r /l_wrist_pose:=/l_goal_pose \
  -r /r_wrist_pose:=/r_goal_pose
