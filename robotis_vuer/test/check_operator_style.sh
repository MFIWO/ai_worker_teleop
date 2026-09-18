#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws/src/robotis_applications
ament_flake8 robotis_vuer/robotis_vuer/operator_console.py robotis_vuer/robotis_vuer/operator_hand.py robotis_vuer/robotis_vuer/loop_keys.py robotis_vuer/robotis_vuer/start_operator_console.py robotis_vuer/test/test_operator_console.py robotis_vuer/test/test_operator_hand.py
