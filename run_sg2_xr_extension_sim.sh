#!/usr/bin/env bash
# [ADDED] Near-extension trial; preserves old XR and fixed-resume launchers.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test -f /opt/ros/jazzy/setup.bash || { echo '[ERROR] Run inside robotis-applications Docker'; exit 1; }
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
test "$ROS_DOMAIN_ID" = 231 || { echo '[ERROR] SIM trial requires domain231'; exit 1; }
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export PYTHONPATH="$HERE/robotis_vuer:${PYTHONPATH:-}"
echo '[EXTENSION] AI Worker must use: bash /workspace/sg2_xr_profiles/run_extension_controller.sh'
echo '[EXTENSION] That launcher selects the separate model and compiled QP precision overlay.'
echo '[EXTENSION] First X+A calibrates; later X+A resumes SAME mapping. Y+B stops.'
echo '[EXTENSION] Wait for [XR RESUME READY]. Startup hand drift holds goals, not a new calibration.'
echo '[EXTENSION] If [XR RESUME WAIT_START] appears: realign, release X/A, then X+A to begin.'
echo '[EXTENSION] Y+B still stops immediately. No fingers/base/lift commands.'
exec python3 -m robotis_vuer.vr_publisher_sg2_xr_extension --ros-args -r __node:=sg2_xr_publisher "$@"
