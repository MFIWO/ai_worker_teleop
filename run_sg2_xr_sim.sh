#!/usr/bin/env bash
# [ADDED] Opt-in XR wrist mode. Run INSIDE robotis-applications Docker.
# This does not launch a follower, prepare arms, install packages or enable VR.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo '[ERROR] Run this script inside the robotis-applications container.' >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
if [[ "$ROS_DOMAIN_ID" != 231 ]]; then
    echo '[ERROR] This simulation launcher requires ROS_DOMAIN_ID=231. It does not use robot domain 30.' >&2
    exit 1
fi
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export PYTHONPATH="$HERE/robotis_vuer:${PYTHONPATH:-}"
echo '[XR MODE] Stop the old Vuer process first. Keep Isaac Sim unchanged.'
echo '[XR MODE] AI Worker command needs: controller_type:=vr hand:=false arm:=false'
echo '[XR MODE] Hold comfortable arms; release X/A, then X+A to calibrate. Y+B stops.'
exec python3 -m robotis_vuer.vr_publisher_sg2_xr --ros-args -r __node:=sg2_xr_publisher "$@"
