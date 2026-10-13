#!/usr/bin/env bash
# [ADDED] Run on the robot HOST with this branch checked out; COPY ONLY, never launches motors.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${AI_WORKER_CONTAINER:-ai_worker}"
DEST=/workspace/ai_worker_xr
docker exec "$CONTAINER" mkdir -p "$DEST/hardware" "$DEST/sim"
# Copy named source files only: no .git, credentials, logs, built overlays or runtime profiles.
for name in __init__.py xr_contract.py make_profile.py command_gate.py capture_snapshot.py \
            sh5_model.py collision_primitives.py check_collision.py hardware.launch.py run_controller.sh log_arms.sh; do
  test -f "$HERE/$name" || { echo "[ERROR] Missing release file: $name"; exit 1; }
done
for name in __init__.py xr_contract.py make_profile.py command_gate.py capture_snapshot.py \
            sh5_model.py collision_primitives.py check_collision.py hardware.launch.py run_controller.sh log_arms.sh; do
  docker cp "$HERE/$name" "$CONTAINER:$DEST/hardware/$name"
done
docker cp "$HERE/../sim/log_sg2_arm.py" "$CONTAINER:$DEST/sim/log_sg2_arm.py"
echo "[INSTALLED] $CONTAINER:$DEST/hardware (source only; no controller/motor started)"
echo '[NEXT] Existing hardware follower in known ready pose, then run_controller.sh sg2|sh5.'
