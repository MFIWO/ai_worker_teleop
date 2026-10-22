#!/usr/bin/env bash
# [ADDED] Robot AI Worker Docker. Default passively verifies model/motors before launching.
# The already-working physical follower must be running separately; this script never starts it.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="${1:-}"; shift || true
case "$MODEL" in sg2|sh5) ;; *) echo 'Usage: bash hardware/run_controller.sh sg2|sh5 [--hands] [--check-only [--allow-torque-off]] [--strict-commissioning]'; exit 2;; esac
HANDS=0
MANUAL=0
CHECK=0
ALLOW_TORQUE_OFF=0
READINESS_POLICY=driver_managed  # [CHANGED] Normal follower-driver path, no independent torque proof.
while (( $# )); do
  case "$1" in
    --hands) HANDS=1 ;;
    --manual-controls) MANUAL=1 ;;
    --check-only) CHECK=1 ;;
    --allow-torque-off) ALLOW_TORQUE_OFF=1 ;;
    --strict-commissioning) READINESS_POLICY=strict_torque ;;
    *) echo "[ERROR] Unexpected argument: $1"; exit 2 ;;
  esac
  shift
done
if (( ALLOW_TORQUE_OFF && ! CHECK )); then
  echo '[ERROR] --allow-torque-off is a legacy passive-only option; use --check-only'
  exit 2
fi
if (( HANDS )) && [[ "$MODEL" != sh5 ]]; then
  echo '[ERROR] --hands requires sh5'; exit 2
fi
if (( MANUAL )) && { [[ "$MODEL" != sh5 ]] || (( ! HANDS )); }; then
  echo '[ERROR] --manual-controls requires sh5 --hands'; exit 2
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
BUILDER=build_startup_timing.py
BUILD_ROOT=/workspace/ai_worker_xr_startup_steady
if (( HANDS )); then
  BUILDER=build_sh5_hands.py
  BUILD_ROOT=/workspace/ai_worker_xr_sh5_hands
fi
if (( ! CHECK )); then
  python3 -B "$HERE/hardware/$BUILDER" --verify-only || {
    echo "[ERROR] Build startup node before operating: python3 -B $HERE/hardware/$BUILDER"
    exit 1
  }
fi
BIN="$BUILD_ROOT"/build/cyclo_motion_controller_ros/vr_controller_node
if (( CHECK )); then
  BIN="$(ros2 pkg prefix cyclo_motion_controller_ros)/lib/cyclo_motion_controller_ros/vr_controller_node"
fi
if ldd "$BIN" | grep -q sg2_extension_overlay; then
  echo '[ERROR] Simulation overlay library still selected. Open a fresh robot terminal.'; exit 1
fi
# [CHANGED 2026-10-14] SG2 numerical-convergence fix verified by the stationary
# native-Cyclo test. This is NOT the simulation model/extension overlay.
# Build once before operating the follower; never auto-compile during teleop.
if [[ "$MODEL" == sg2 ]] && (( ! CHECK )); then
  QP_ROOT=/workspace/ai_worker_xr_qp_precision
  python3 -B "$HERE/hardware/build_qp_precision.py" --root "$QP_ROOT" --verify-only || {
    echo '[ERROR] Prepare the SG2 numerical core before starting the follower:'
    echo "  python3 -B $HERE/hardware/build_qp_precision.py"
    exit 1
  }
  QP_LIB="$QP_ROOT/install/cyclo_motion_controller_core/lib"
  export LD_LIBRARY_PATH="$QP_LIB:${LD_LIBRARY_PATH:-}"
  python3 - "$BIN" "$QP_LIB/libcyclo_motion_controller_core.so" <<'PY'
from pathlib import Path
import subprocess, sys
lines = subprocess.check_output(['ldd', sys.argv[1]], text=True).splitlines()
matches = [line.split('=>', 1)[1].strip().split()[0]
           for line in lines if line.strip().startswith('libcyclo_motion_controller_core.so =>')]
if len(matches) != 1 or Path(matches[0]).resolve() != Path(sys.argv[2]).resolve():
    raise SystemExit('[ERROR] SG2 runtime did not resolve the verified numerical core')
print('[QP CORE]', matches[0])
PY
fi
PROFILE="/workspace/ai_worker_xr_profiles/${MODEL}_$(date +%Y%m%d_%H%M%S_%N)"
SNAPSHOT_ARGS=(--readiness-policy "$READINESS_POLICY")  # [CHANGED]
if (( ALLOW_TORQUE_OFF )); then
  # [ADDED] Read-only standby mode; the command gate's live torque guard is untouched.
  SNAPSHOT_ARGS+=(--allow-torque-off)
fi
# [CHANGED] Allow up to 30s for Zenoh discovery + measured-state stabilization.
python3 -m hardware.capture_snapshot --model "$MODEL" --output "$PROFILE/snapshot" --timeout 30 "${SNAPSHOT_ARGS[@]}"
HAND_ARGS=()
if (( HANDS )); then HAND_ARGS+=(--hands); fi
if (( MANUAL )); then HAND_ARGS+=(--manual-controls); fi
python3 -m hardware.make_profile "${HAND_ARGS[@]}" --model "$MODEL" --output "$PROFILE" \
  --description "$PROFILE/snapshot/robot_description.urdf" --snapshot "$PROFILE/snapshot/joints.json" \
  --readiness-policy "$READINESS_POLICY"
export AI_WORKER_XR_PROFILE="$PROFILE/profile.json"
printf '%s\n' "$AI_WORKER_XR_PROFILE" > "/workspace/ai_worker_xr_profiles/current_${MODEL}.txt"
if (( CHECK )); then
  echo "[PASS] preflight only; no controller started. $AI_WORKER_XR_PROFILE"
  if (( ALLOW_TORQUE_OFF )); then
    echo '[STANDBY] Physical motor torque remains unchanged; this is NOT approval to move arms.'
  fi
  exit 0
fi
echo "[HARDWARE] Starting INACTIVE Cyclo/checker/gate; readiness=$READINESS_POLICY. Existing follower was not modified."
exec ros2 launch "$HERE/hardware/hardware.launch.py"
