#!/usr/bin/env bash
# [CHANGED 2026-10-14] --start combines the existing SG2 ready pose and Vuer.
# Original ROBOTIS follower/controller files are NOT modified by this wrapper.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
  echo 'Usage: bash run_xr_hardware_pc.sh sg2|sh5 [--start | --arm] [--hands] [--ready-pose | --ready-only] [--server-ip PC_IPv4] [--robot-ip ROBOT_IP_OR_HOSTNAME] [--strict-commissioning]'
  echo '  --hands: SH5 bare wrists + native hand retarget; use sh5 --arm --hands.'
  echo '  --input controllers: SH5 SG2-style Quest controllers, proportional grasp and three-finger pinch.'
  echo '  --hand-mode 1|2: hands: ROBOTIS or xr_ai_teleop HX5 DexPilot; controllers: trigger/grip or button-selected poses.'
  echo '  --start: SG2 only; existing ready-pose move, then armed Vuer (manual X+A still required).'
  echo '  --server-ip: PC address used by Quest; validate its TLS certificate before any ready move.'
  echo '  --robot-ip: robot Zenoh router address; this is NOT the Vuer certificate address.'
  echo '  Order: follower first -> this PC command -> robot run_controller.sh after READY.'
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
MODEL="${1:-}"; shift || true
case "$MODEL" in sg2|sh5) ;; *) usage; exit 2;; esac
export AI_WORKER_XR_ALLOW_ARM=0
HANDS=0
PREPARE_READY=0
READY_ONLY=0       # [ADDED] Move to measured ready pose, exit BEFORE starting Vuer.
ROBOT_ROUTER=""     # [ADDED] Explicit robot Zenoh router; no manual shell override needed.
SERVER_IP=""        # [CHANGED] Quest HTTPS server is the PC, not the robot router.
READINESS_POLICY=driver_managed  # [CHANGED] Normal path; does not claim torque is ON.
while (( $# )); do
  case "$1" in
    --start) PREPARE_READY=1; export AI_WORKER_XR_ALLOW_ARM=1 ;; # [CHANGED] Explicit motion opt-in.
    --arm) export AI_WORKER_XR_ALLOW_ARM=1 ;;
    --hands) HANDS=1 ;;
    --hand-mode)
      [[ "${2:-}" == 1 || "${2:-}" == 2 ]] || { echo '[ERROR] --hand-mode 1|2'; exit 2; }
      export AI_WORKER_SH5_HAND_MODE="$2"; shift ;;
    --input)
      [[ "${2:-}" == hands || "${2:-}" == controllers ]] || { echo '[ERROR] --input hands|controllers'; exit 2; }
      export AI_WORKER_SH5_INPUT="$2"; HANDS=1; shift ;;
    --ready-pose) PREPARE_READY=1 ;;
    --ready-only) READY_ONLY=1; PREPARE_READY=1 ;;
    --strict-commissioning) READINESS_POLICY=strict_torque ;;
    --server-ip)
      if (( $# < 2 )) || [[ -z "$2" || "$2" == --* ]]; then
        echo '[ERROR] --server-ip needs the PC IPv4 address used by Quest'; exit 2
      fi
      SERVER_IP="$2"
      shift
      ;;
    --robot-ip)
      if (( $# < 2 )) || [[ -z "$2" || "$2" == --* ]]; then
        echo '[ERROR] --robot-ip needs the actual robot IP/hostname'; exit 2
      fi
      ROBOT_ROUTER="$2"
      shift
      ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[ERROR] Unknown argument: $1"; exit 2 ;;
  esac
  shift
done
if (( HANDS )) && [[ "$MODEL" != sh5 ]]; then
  echo "[ERROR] --hands requires sh5"; exit 2
fi
if (( PREPARE_READY )) && [[ "$MODEL" != sg2 ]]; then
  echo '[ERROR] --ready-pose is SG2-only; no verified SH5 ready-pose mover'; exit 2
fi
if (( READY_ONLY )) && [[ "$AI_WORKER_XR_ALLOW_ARM" == 1 ]]; then
  echo '[ERROR] --ready-only cannot be combined with --arm/--start; the XR node will not start'; exit 2
fi
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
[[ "$ROS_DOMAIN_ID" == 30 ]] || { echo '[ERROR] Real robot uses domain30; stop simulation modes first'; exit 1; }
# [CHANGED] Respect the selected RMW (official AI Worker v2 uses Zenoh).
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export AI_WORKER_XR_EXECUTION=hardware AI_WORKER_XR_MODEL="$MODEL"
export AI_WORKER_XR_READINESS_POLICY="$READINESS_POLICY"  # [CHANGED] Require matching robot profile.
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

# [CHANGED] Local TLS validation only; no ROS nodes, trust-store writes, or network
# connections. Do it BEFORE a requested ready move, not after moving the robot.
if [[ -n "$SERVER_IP" ]] && (( ! READY_ONLY )); then
  python3 -m hardware.vuer_tls \
    --cert "$HERE/robotis_vuer/robotis_vuer/cert.pem" \
    --key "$HERE/robotis_vuer/robotis_vuer/key.pem" --server-ip "$SERVER_IP" || {
    echo '[ERROR] Vuer TLS check failed; no ready-pose command sent.'
    echo '[FIX] In PC Docker: cd /root && bash /root/gen_cert.sh <PC_IP>'
    exit 1
  }
  echo "[QUEST URL] https://${SERVER_IP}:8012?ws=wss://${SERVER_IP}:8012"
fi

if (( PREPARE_READY )); then
  # [ADDED] The legacy mover sends BOTH arms to [0,0,0,-1.57,0,0,0], not elbow only.
  # Run only on explicit request, before Cyclo/VR is started; do not enable torque here.
  [[ -f "$HERE/robotis_vuer/robotis_vuer/prepare_sg2_ready_pose.py" ]] || {
    echo '[ERROR] Missing prepare_sg2_ready_pose.py; no motion sent'; exit 1;
  }
  echo '[READY POSE] SG2 both seven-joint arms to [0,0,0,-1.57,0,0,0] in >=5s.'
  echo '[READY POSE] Stop old Cyclo/VR command publishers; keep physical follower running.'
  # [FIX 2026-10-14] Check Python message imports before starting a ROS node.
  python3 -c 'from control_msgs.action import FollowJointTrajectory; from action_msgs.msg import GoalStatus' || {
    echo '[ERROR] PC Docker needs ros-jazzy-control-msgs and ros-jazzy-action-msgs; no motion sent'
    exit 1
  }
  # [CHANGED] Validate real motor READ health; torque management stays with the
  # original follower. The unsound buffered-register service is never polled.
  [[ -f "$HERE/hardware/ready_pose_precheck.py" ]] || {
    echo '[ERROR] Missing hardware/ready_pose_precheck.py; sync the new MFIWO PC helper'
    exit 1
  }
  python3 -m hardware.ready_pose_precheck --timeout 30 --readiness-policy "$READINESS_POLICY" || {
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
  echo '[READY POSE COMPLETE] Existing measured ready pose reached; starting Vuer in THIS terminal.'
  echo '[NEXT: ROBOT] Now start: bash /workspace/ai_worker_xr/hardware/run_controller.sh sg2'
  if [[ "$READINESS_POLICY" == strict_torque ]]; then
    echo '[NEXT: ROBOT] Append --strict-commissioning to match this PC policy.'
  fi
  echo '[NEXT: QUEST] Wait for controller/model feedback, then press X+A. No automatic VR activation.'
fi

echo "[HARDWARE $MODEL] readiness=$READINESS_POLICY; RMW=$RMW_IMPLEMENTATION; Cartesian filter=3m/s/15rad/s; URDF joint limits."
if (( ! HANDS )); then
  echo '[HARDWARE] Manual X+A starts/resumes; Y+B stops. SG2 sticks/trigger via gate.'
fi
PUBLISHER=robotis_vuer.vr_publisher_ai_worker_hardware
if (( HANDS )); then
  PUBLISHER=robotis_vuer.vr_publisher_sh5_hands_hardware
  if [[ "${AI_WORKER_SH5_INPUT:-hands}" == controllers ]]; then
    echo '[SH5 CONTROLLERS] X+A start, Y+B stop; trigger=proportional grasp; grip=thumb/index/middle pinch.'
  else
    echo '[SH5 HANDS] Bare wrists + SH5 fingers. Pinch + opposite squeeze for 3s starts/stops.'
  fi
fi
exec python3 -m "$PUBLISHER" --ros-args -r __node:=ai_worker_xr_hardware
