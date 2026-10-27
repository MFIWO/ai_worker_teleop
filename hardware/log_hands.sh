#!/usr/bin/env bash
# Passive ROS subscriptions only. Never opens a serial port or sends motor commands.
set -eo pipefail
SECONDS_TO_RECORD=120
DRIVER_TRACE=0
while (( $# )); do
  case "$1" in
    --seconds)
      [[ "${2:-}" =~ ^[0-9]+$ ]] || { echo 'Expected --seconds 10..600'; exit 2; }
      SECONDS_TO_RECORD="$2"; shift ;;
    --driver-trace) DRIVER_TRACE=1 ;;
    *) echo 'Usage: log_hands.sh [--seconds 10..600] [--driver-trace]'; exit 2 ;;
  esac
  shift
done
(( SECONDS_TO_RECORD >= 10 && SECONDS_TO_RECORD <= 600 )) || exit 2
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export PYTHONPATH="$HERE:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
CAPTURE_DIR="/workspace/ai_worker_xr_logs/hand_mode1_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$CAPTURE_DIR"
TOPICS=(
  /left_hand/hand_joint_pos /right_hand/hand_joint_pos
  /xr_hardware/sh5/native_left_hand /xr_hardware/sh5/native_right_hand
  /leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory
  /leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory
  /hand_l_controller/controller_state /hand_r_controller/controller_state
  /joint_states /ffw_follower/dxl_state
  /xr_hardware/intent /xr_hardware/gate_status /reactivate
  /leader/joint_trajectory_command_broadcaster_left/joint_trajectory
  /leader/joint_trajectory_command_broadcaster_right/joint_trajectory
  /rosout /diagnostics
)
python3 - "$CAPTURE_DIR" "$HERE" "$SECONDS_TO_RECORD" "${TOPICS[@]}" <<'PY'
import datetime, hashlib, json, re, sys
from pathlib import Path
dest, root = map(Path, sys.argv[1:3])
manifest = {'observer_only': True, 'requested_mode': 1, 'seconds': int(sys.argv[3]),
            'start_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'topics': sys.argv[4:], 'processes': [], 'source_sha256': {},
            'limitations': ['ROS timestamps do not establish physical motor timing',
                           'source clocks may differ; prefer robot bag receipt timestamps',
                           'observer uses best-effort subscriptions; missing records may be capture loss']}
for p in Path('/proc').iterdir():
    if not p.name.isdigit():
        continue
    try:
        raw = (p/'cmdline').read_bytes().split(b'\0')
        args = [a.decode(errors='replace') for a in raw if a]
        if not args:
            continue
        if (Path(args[0]).name in ('ros2_control_node', 'retargeting_teleop', 'vr_controller_node')
                or any(a in ('hardware.robot_session', 'hardware.cyclo_hand_retarget_node',
                             'hardware.robotis_hand_retarget_node', 'hardware.xr_hand_retarget_node',
                             'hardware.command_gate') for a in args)):
            libraries = sorted(set(line.split()[-1] for line in (p/'maps').read_text().splitlines()
                                   if 'libdynamixel_sdk' in line or 'libdynamixel_hardware_interface' in line))
            manifest['processes'].append({'pid': p.name, 'args': args, 'libraries': libraries})
    except (OSError, UnicodeError):
        pass
package = Path('/root/ros2_ws/install/dynamixel_sdk/share/dynamixel_sdk/package.xml')
if package.exists():
    manifest['default_sdk_version'] = re.search(r'<version>(.*?)</version>', package.read_text()).group(1)
for name in ('hardware/hardware.launch.py', 'hardware/cyclo_hand_retarget_node.py',
             'hardware/sh5_controller_hands.py'):
    path = root/name
    if path.exists():
        manifest['source_sha256'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
sessions = sorted(Path('/workspace/ai_worker_xr_sessions').glob('[0-9]*'), key=lambda p: p.name)
if sessions:
    manifest['latest_session_directory'] = str(sessions[-1])
(dest/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
qos = ''.join(f'{topic}:\n  reliability: best_effort\n  durability: volatile\n'
              '  history: keep_last\n  depth: 200\n' for topic in manifest['topics'])
(dest/'qos.yaml').write_text(qos)
(dest/'storage.yaml').write_text('write:\n  pragmas: ["journal_mode = WAL", "synchronous = NORMAL"]\n')
PY
echo "[HAND LOG] Output: $CAPTURE_DIR"
echo '[HAND LOG] Passive observation only; waiting for ROS recorder readiness.'
TRACE_OPTION=()
if (( DRIVER_TRACE )); then TRACE_OPTION=(--driver-trace); fi
exec python3 -B -m hardware.hand_capture_runner --output "$CAPTURE_DIR" \
  --seconds "$SECONDS_TO_RECORD" "${TRACE_OPTION[@]}" --topics "${TOPICS[@]}"
