#!/usr/bin/env bash
# Run in the PC robotis-applications container, in a separate terminal.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$script_dir/robotis_vuer/robotis_vuer/observe_arm_pose.py" "$@"
