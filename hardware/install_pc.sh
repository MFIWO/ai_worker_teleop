#!/usr/bin/env bash
# Copy PC Python sources only; preserve existing certificates and installed packages.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${AI_WORKER_PC_CONTAINER:-robotis-applications}"
DEST="${AI_WORKER_PC_SOURCE:-/root/ros2_ws/src/robotis_applications}"
docker exec "$CONTAINER" mkdir -p "$DEST/hardware" "$DEST/robotis_vuer/robotis_vuer"
for source in "$HERE"/robotis_vuer/robotis_vuer/*.py; do
  docker cp "$source" "$CONTAINER:$DEST/robotis_vuer/robotis_vuer/$(basename "$source")"
done
for name in __init__.py xr_contract.py aux_controls.py sh5_model.py sh5_hands_model.py vuer_tls.py; do
  docker cp "$HERE/hardware/$name" "$CONTAINER:$DEST/hardware/$name"
done
docker cp "$HERE/run_xr_hardware_pc.sh" "$CONTAINER:$DEST/run_xr_hardware_pc.sh"
echo "[INSTALLED] PC source: $CONTAINER:$DEST"
echo '[NEXT] run_xr_hardware_pc.sh sh5 --arm --hands --server-ip PC_IP --robot-ip ROBOT_IP'
