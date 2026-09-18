#!/usr/bin/env bash
# Run in the existing PC container, in the one local terminal used for operation.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PYTHONPATH="$script_dir/robotis_vuer:${PYTHONPATH:-}"
exec python3 "$script_dir/robotis_vuer/robotis_vuer/operator_console.py" "$@"
