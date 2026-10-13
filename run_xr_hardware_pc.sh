#!/usr/bin/env bash
# [CHANGED 2026-10-14] Explicit SG2 ready-pose opt-in before hardware XR publisher.
# Original ROBOTIS follower/controller files are NOT modified by this wrapper.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL="${1:-}"; shift || true
case "$MODEL" in sg2|sh5) ;; *) echo 'Usage: bash run_xr_hardware_pc.sh sg2|sh5 [--arm] [--ready-pose] [--ready-only] [--robot-ip IPv4_OR_HOSTNAME]'; exit 2;; esac
export AI_WORKER_XR_ALLOW_ARM=0
PREPARE_READY=0
READY_ONLY=0       # [ADDED] Move to measured ready pose, exit BEFORE starting Vuer.
ROBOT_ROUTER=""     # [ADDED] Explicit robot Zenoh router; no manual shell override needed.
while (( $# )); do
  case "$1" in
    --arm) export AI_WORKER_XR_ALLOW_ARM=1 ;;
    --ready-pose) PREPARE_READY=1 ;;
    --ready-only) READY_ONLY=1; PREPARE_READY=1 ;;
    --robot-ip)
      if (( $# < 2 )) || [[ -z "$2" ]]; then
        echo '[ERROR] --robot-ip needs the actual robot IP/hostname'; exit 2
      fi
      ROBOT_ROUTER="$2"
      shift
      ;;
    *) echo "[ERROR] Unknown argument: $1"; exit 2 ;;
  esac
  shift
done
if (( PREPARE_READY )) && [[ "$MODEL" != sg2 ]]; then
  echo '[ERROR] --ready-pose is SG2-only; no verified SH5 ready-pose mover'; exit 2
fi
if (( READY_ONLY )) && [[ "$AI_WORKER_XR_ALLOW_ARM" == 1 ]]; then
  echo '[ERROR] --ready-only cannot be combined with --arm; the XR node will not start'; exit 2
fi
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
[[ "$ROS_DOMAIN_ID" == 30 ]] || { echo '[ERROR] Real robot uses domain30; stop simulation modes first'; exit 1; }
# [CHANGED] Respect the selected RMW (official AI Worker v2 uses Zenoh).
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export AI_WORKER_XR_EXECUTION=hardware AI_WORKER_XR_MODEL="$MODEL"
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export PYTHONPATH="$HERE:$HERE/robotis_vuer:${PYTHONPATH:-}"
# [ADDED] Use the actual robot's router when --robot-ip is supplied.
# Never silently override an operator-provided Zenoh config or accept shell metacharacters.
if [[ -n "$ROBOT_ROUTER" ]]; then
  if [[ ! "$ROBOT_ROUTER" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo '[ERROR] Invalid --robot-ip (IPv4 or DNS hostname only)'; exit 2
  fi
  if [[ -n "${ZENOH_CONFIG_OVERRIDE:-}" ]]; then
    echo '[ERROR] Existing ZENOH_CONFIG_OVERRIDE set; unset it before using --robot-ip'; exit 2
  fi
  export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${ROBOT_ROUTER}:7447\"]"
  export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-5}"
  echo "[ZENOH] Connecting to explicitly selected robot router: $ROBOT_ROUTER:7447"
elif [[ "$RMW_IMPLEMENTATION" == rmw_zenoh_cpp && -z "${ZENOH_CONFIG_OVERRIDE:-}" ]]; then
  echo '[WARN] No robot router specified. Use --robot-ip <ROBOT_IP> or an existing ZENOH_CONFIG_OVERRIDE.'
fi

if (( PREPARE_READY )); then
  # [ADDED] The legacy mover sends BOTH arms to [0,0,0,-1.57,0,0,0], not elbow only.
  # Run only on explicit request, before Cyclo/VR is started; do not enable torque here.
  [[ -f "$HERE/robotis_vuer/robotis_vuer/prepare_sg2_ready_pose.py" ]] || {
    echo '[ERROR] Missing prepare_sg2_ready_pose.py; no motion sent'; exit 1;
  }
  echo '[READY POSE] SG2 both seven-joint arms to [0,0,0,-1.57,0,0,0] in >=5s.'
  echo '[READY POSE] Stop old Cyclo/VR command publishers; keep physical follower running.'
  # [CHANGED] One read-only precheck; the upstream torque_state topic may be
  # unreliable. Require fresh DXL bus, synchronized clocks, and either
  # reliable ON telemetry or positive reads of all 14 Torque Enable registers.
  # Do NOT send arm commands when hardware proof is unavailable.
  [[ -f "$HERE/hardware/ready_pose_precheck.py" ]] || {
    echo '[ERROR] Missing hardware/ready_pose_precheck.py; sync the new MFIWO PC helper'
    exit 1
  }
  python3 -m hardware.ready_pose_precheck --timeout 30 || {
    echo '[ERROR] SG2 ready-pose check failed; Vuer will NOT start'
    exit 1
  }
  # Existing mover also checks fresh measured joints, both action servers,
  # other publishers, stationary arms, completion, and measured settling.
  python3 "$HERE/robotis_vuer/robotis_vuer/prepare_sg2_ready_pose.py" --duration 5.0 || {
    echo '[ERROR] Ready-pose move failed; hardware XR publisher NOT started'; exit 1;
  }
  if (( READY_ONLY )); then
    echo '[READY POSE] Measured SG2 ready pose reached. No VR publisher started.'
    echo '[NEXT] Start robot run_controller.sh sg2, then launch this PC script with sg2 --arm (no --ready-pose).'
    exit 0
  fi
  echo '[READY POSE] Complete; starting XR publisher.'
fi

echo "[HARDWARE $MODEL] gate preflight + manual X+A required. Y+B stops. Rate=.06m/s."
echo "[HARDWARE] RMW=$RMW_IMPLEMENTATION. No trigger/hand/base/lift commands; do not run legacy Vuer beside this process."
exec python3 -m robotis_vuer.vr_publisher_ai_worker_hardware --ros-args -r __node:=ai_worker_xr_hardware
