#!/usr/bin/env bash
# Controller teleop with the same SG2 90-degree elbow ready pose as hand mode.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
echo 'Controller mode: preparing SG2 elbows at about 90 degrees.'
echo 'Keep follower running; start the robot VR controller only AFTER READY.'
python3 "$script_dir/robotis_vuer/robotis_vuer/prepare_sg2_ready_pose.py" --duration 5.0
exec ros2 launch robotis_vuer vr.launch.py model:=sg2
