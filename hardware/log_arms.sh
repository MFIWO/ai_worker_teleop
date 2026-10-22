#!/usr/bin/env bash
# [ADDED] Passive hardware log; use the exact model selected by run_controller.sh.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="${1:-}"
case "$MODEL" in sg2|sh5) ;; *) echo 'Usage: log_arms.sh sg2|sh5'; exit 2;; esac
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
if [[ -d /opt/venv/lib/python3.12/site-packages ]]; then export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"; fi
PROFILE="$(cat "/workspace/ai_worker_xr_profiles/current_${MODEL}.txt")"
URDF="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["urdf"])' "$PROFILE")"
OUT="/workspace/ai_worker_xr_logs/${MODEL}_$(date +%Y%m%d_%H%M%S_%N)"
exec python3 -u "$HERE/sim/log_sg2_arm.py" --hardware --urdf "$URDF" --output "$OUT" --hz 10
