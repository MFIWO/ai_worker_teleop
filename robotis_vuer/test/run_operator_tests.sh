#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws/src/robotis_applications
export ROS_DOMAIN_ID=231
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$PWD/robotis_vuer:${PYTHONPATH:-}"
python3 -m unittest discover -s robotis_vuer/test -p 'test_operator_*.py' -v
export PYTHONPATH="$PWD/robotis_vuer/test:$PYTHONPATH"
python3 -m unittest test_arm_tracking_guard test_arm_tracking_integration test_gravity_arm_frame test_sg2_activation test_sg2_ready_pose -v
