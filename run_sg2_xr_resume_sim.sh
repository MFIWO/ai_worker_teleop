#!/usr/bin/env bash
# [ADDED] Fixed-reference restart mode; old run_sg2_xr_sim.sh is preserved.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test -f /opt/ros/jazzy/setup.bash || { echo '[ERROR] Run inside robotis-applications Docker.' >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
test "$ROS_DOMAIN_ID" = 231 || { echo '[ERROR] Simulation domain231 required.' >&2; exit 1; }
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export PYTHONPATH="$HERE/robotis_vuer:${PYTHONPATH:-}"
echo '[FIXED RESUME] Keep existing sim/Cyclo shoulder_free commands. Stop old Vuer first.'
echo '[FIXED RESUME] First X+A calibrates. Later X+A resumes same reference only after hand alignment.'
echo '[FIXED RESUME] Wait for XR RESUME READY; Y+B stops. Restart publisher explicitly for a NEW calibration.'
exec python3 -m robotis_vuer.vr_publisher_sg2_xr_resume --ros-args -r __node:=sg2_xr_resume_publisher "$@"
