#!/usr/bin/env bash
# [ADDED] PC/ROBOTIS Applications; no simulation domain and no automatic ready-pose movement.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL="${1:-}"; shift || true
case "$MODEL" in sg2|sh5) ;; *) echo 'Usage: bash run_xr_hardware_pc.sh sg2|sh5 [--arm]'; exit 2;; esac
export AI_WORKER_XR_ALLOW_ARM=0
if [[ "${1:-}" == --arm ]]; then export AI_WORKER_XR_ALLOW_ARM=1; shift; fi
if (( $# )); then echo '[ERROR] Unknown arguments; runtime rate override is not a hardware commissioning option'; exit 2; fi
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
[[ "$ROS_DOMAIN_ID" == 30 ]] || { echo '[ERROR] Real robot uses domain30; stop simulation modes first'; exit 1; }
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp AI_WORKER_XR_EXECUTION=hardware AI_WORKER_XR_MODEL="$MODEL"
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export PYTHONPATH="$HERE:$HERE/robotis_vuer:${PYTHONPATH:-}"
echo "[HARDWARE $MODEL] gate preflight + manual X+A required. Y+B stops. Rate=.06m/s."
echo '[HARDWARE] No trigger/hand/base/lift commands; do not run legacy Vuer beside this process.'
exec python3 -m robotis_vuer.vr_publisher_ai_worker_hardware --ros-args -r __node:=ai_worker_xr_hardware
