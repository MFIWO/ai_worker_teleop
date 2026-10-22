#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export PYTHONPATH="$HERE:$HERE/robotis_vuer:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp
exec python3 -m hardware.pc_session "$@"
