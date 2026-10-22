#!/usr/bin/env bash
# [ADDED] Same extension controller; optional passthrough calibration display.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test -f /opt/ros/jazzy/setup.bash || { echo '[ERROR] Run inside robotis-applications Docker'; exit 1; }
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
test "$ROS_DOMAIN_ID" = 231 || { echo '[ERROR] SG2 simulation domain231 required'; exit 1; }
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export PYTHONPATH="$HERE/robotis_vuer:${PYTHONPATH:-}"
echo '[CAL OVERLAY] Stop the previous Vuer process; no second publisher on port8012.'
echo '[CAL OVERLAY] Isaac Sim and AI Worker extension-controller commands unchanged.'
echo '[CAL OVERLAY] Quest: refresh Vuer, use the usual Enter VR/passthrough, then align dots AND axes.'
echo '[CAL OVERLAY] First X+A captures; WAIT_START needs aligned fresh X+A; Y+B stops.'
exec python3 -m robotis_vuer.vr_publisher_sg2_xr_guided --ros-args -r __node:=sg2_xr_publisher "$@"
