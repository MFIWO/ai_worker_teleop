#!/usr/bin/env bash
# [ADDED] Robot AI Worker Docker. Default passively verifies model/motors before launching.
# The already-working physical follower must be running separately; this script never starts it.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="${1:-}"; shift || true
case "$MODEL" in sg2|sh5) ;; *) echo 'Usage: bash hardware/run_controller.sh sg2|sh5 [--check-only]'; exit 2;; esac
CHECK=0
if [[ "${1:-}" == --check-only ]]; then CHECK=1; shift; fi
(( $# == 0 )) || { echo '[ERROR] Unexpected arguments'; exit 2; }
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
[[ "$ROS_DOMAIN_ID" == 30 ]] || { echo '[ERROR] hardware/domain30 only'; exit 1; }
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp AI_WORKER_XR_EXECUTION=hardware
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS CYCLO_SG2_QP_PRECISION
if [[ -d /opt/venv/lib/python3.12/site-packages ]]; then
  export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi
export PYTHONPATH="$HERE:$HERE/robotis_vuer:${PYTHONPATH:-}"
python3 - <<'PY'
import importlib.util
required=('numpy','scipy','yaml','pinocchio','rclpy','dynamixel_interfaces')
missing=[name for name in required if importlib.util.find_spec(name) is None]
if missing:raise SystemExit('[ERROR] Missing robot Python dependencies: '+', '.join(missing))
PY
BIN="$(ros2 pkg prefix cyclo_motion_controller_ros)/lib/cyclo_motion_controller_ros/vr_controller_node"
if ldd "$BIN" | grep -q sg2_extension_overlay; then
  echo '[ERROR] Simulation overlay library still selected. Open a fresh robot terminal.'; exit 1
fi
PROFILE="/workspace/ai_worker_xr_profiles/${MODEL}_$(date +%Y%m%d_%H%M%S_%N)"
python3 -m hardware.capture_snapshot --model "$MODEL" --output "$PROFILE/snapshot"
python3 -m hardware.make_profile --model "$MODEL" --output "$PROFILE" \
  --description "$PROFILE/snapshot/robot_description.urdf" --snapshot "$PROFILE/snapshot/joints.json"
export AI_WORKER_XR_PROFILE="$PROFILE/profile.json"
printf '%s\n' "$AI_WORKER_XR_PROFILE" > "/workspace/ai_worker_xr_profiles/current_${MODEL}.txt"
if (( CHECK )); then echo "[PASS] preflight only; no controller started. $AI_WORKER_XR_PROFILE"; exit 0; fi
echo '[HARDWARE] Starting INACTIVE Cyclo/checker/gate. Existing follower was not modified.'
exec ros2 launch "$HERE/hardware/hardware.launch.py"
