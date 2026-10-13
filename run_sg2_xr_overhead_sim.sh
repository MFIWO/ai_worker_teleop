#!/usr/bin/env bash
# [ADDED] SIM-only overhead + straight calibration-ray targets + existing AR guides.
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
echo '[OVERHEAD] Replace old Vuer process; same port8012 and AR calibration guides.'
echo '[OVERHEAD] Isaac Sim and AI Worker extension controller remain unchanged.'
echo '[OVERHEAD] No0.5m clutch stop; finite robot workspace, tracking, joint/CBF checks remain.'
echo '[OVERHEAD] First X+A calibrates; WAIT_START needs aligned fresh X+A; Y+B stops.'
exec python3 -m robotis_vuer.vr_publisher_sg2_xr_overhead --ros-args -r __node:=sg2_xr_publisher "$@"
