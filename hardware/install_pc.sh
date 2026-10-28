#!/usr/bin/env bash
# Copy PC Python sources only; preserve existing certificates and installed packages.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${AI_WORKER_PC_CONTAINER:-robotis-applications}"
DEST="${AI_WORKER_PC_SOURCE:-/root/ros2_ws/src/robotis_applications}"
if [[ "$(stat -Lc '%d:%i' "$HERE")" == "$(docker exec "$CONTAINER" stat -Lc '%d:%i' "$DEST" 2>/dev/null)" ]]; then
  echo "[INSTALLED] PC source is already bind-mounted: $CONTAINER:$DEST"
  echo '[NEXT] bash teleop.sh pc --pc-ip PC_IP --robot-ip ROBOT_IP --loop vr'
  exit 0
fi
docker exec "$CONTAINER" mkdir -p "$DEST/hardware" "$DEST/robotis_vuer/robotis_vuer"
for source in "$HERE"/robotis_vuer/robotis_vuer/*.py; do
  docker cp "$source" "$CONTAINER:$DEST/robotis_vuer/robotis_vuer/$(basename "$source")"
done
for name in __init__.py xr_contract.py aux_controls.py sh5_model.py sh5_hands_model.py vuer_tls.py \
            keyboard_controls.py teleop_cli.py pc_session.py session_processes.py ready_pose.py host_lease.py controller_hand_modes.py controller_stick_motion.py xr_hand_input.py; do
  docker cp "$HERE/hardware/$name" "$CONTAINER:$DEST/hardware/$name"
done
docker cp "$HERE/hardware/run_pc_session.sh" "$CONTAINER:$DEST/hardware/run_pc_session.sh"
docker cp "$HERE/run_xr_hardware_pc.sh" "$CONTAINER:$DEST/run_xr_hardware_pc.sh"
echo "[INSTALLED] PC source: $CONTAINER:$DEST"
echo '[NEXT] run_xr_hardware_pc.sh sh5 --arm --hands --server-ip PC_IP --robot-ip ROBOT_IP'
