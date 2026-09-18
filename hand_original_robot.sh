#!/usr/bin/env bash
# Pre-backport arm routing; finger retargeting stays disabled for SG2.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30
exec ros2 launch \
  /workspace/quest_sg2_teleop/original/cyclo_motion_controller_ros/launch/ai_worker_controller.launch.py \
  controller_type:=vr hand:=false
