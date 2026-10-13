#!/usr/bin/env bash
# [ADDED] Robot AI Worker Docker. Default passively verifies model/motors before launching.
# The already-working physical follower must be running separately; this script never starts it.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="${1:-}"; shift || true
case "$MODEL" in sg2|sh5) ;; *) echo 'Usage: bash hardware/run_controller.sh sg2|sh5 [--check-only [--allow-torque-off]]'; exit 2;; esac
CHECK=0
ALLOW_TORQUE_OFF=0
while (( $# )); do
  case "$1" in
    --check-only) CHECK=1 ;;
    --allow-torque-off) ALLOW_TORQUE_OFF=1 ;;
    *) echo "[ERROR] Unexpected argument: $1"; exit 2 ;;
  esac
  shift
done
if (( ALLOW_TORQUE_OFF && ! CHECK )); then
  echo '[ERROR] --allow-torque-off requires --check-only; live controller still requires torque ON'
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
[[ "$ROS_DOMAIN_ID" == 30 ]] || { echo '[ERROR] hardware/domain30 only'; exit 1; }
# [CHANGED] ROBOTIS AI Worker v2 follower uses Zenoh; never force Fast DDS here.
export RMW_IMPLEMENTATION=rmw_zenoh_cpp AI_WORKER_XR_EXECUTION=hardware
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
SNAPSHOT_ARGS=()
if (( ALLOW_TORQUE_OFF )); then
  # [ADDED] Read-only standby mode; the command gate's live torque guard is untouched.
  SNAPSHOT_ARGS+=(--allow-torque-off)
fi
# [CHANGED] Allow up to 30s for Zenoh discovery + measured-state stabilization.
python3 -m hardware.capture_snapshot --model "$MODEL" --output "$PROFILE/snapshot" --timeout 30 "${SNAPSHOT_ARGS[@]}"
python3 -m hardware.make_profile --model "$MODEL" --output "$PROFILE" \
  --description "$PROFILE/snapshot/robot_description.urdf" --snapshot "$PROFILE/snapshot/joints.json"
export AI_WORKER_XR_PROFILE="$PROFILE/profile.json"
printf '%s\n' "$AI_WORKER_XR_PROFILE" > "/workspace/ai_worker_xr_profiles/current_${MODEL}.txt"
if (( CHECK )); then
  echo "[PASS] preflight only; no controller started. $AI_WORKER_XR_PROFILE"
  if (( ALLOW_TORQUE_OFF )); then
    echo '[STANDBY] Physical motor torque remains unchanged; this is NOT approval to move arms.'
  fi
  exit 0
fi
echo '[HARDWARE] Starting INACTIVE Cyclo/checker/gate. Existing follower was not modified.'
exec ros2 launch "$HERE/hardware/hardware.launch.py"
