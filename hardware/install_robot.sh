#!/usr/bin/env bash
# [ADDED] Run on the robot HOST with this branch checked out; COPY ONLY, never launches motors.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${AI_WORKER_CONTAINER:-ai_worker}"
DEST=/workspace/ai_worker_xr
docker exec "$CONTAINER" mkdir -p "$DEST/hardware" "$DEST/sim"
# Copy named source files only: no .git, credentials, logs, built overlays or runtime profiles.
for name in __init__.py xr_contract.py aux_controls.py make_profile.py command_gate.py capture_snapshot.py ready_pose_precheck.py torque_evidence.py build_qp_precision.py build_startup_timing.py \
            sh5_model.py collision_primitives.py check_collision.py hardware.launch.py run_controller.sh \
            run_commissioning_controller.sh log_arms.sh; do
  test -f "$HERE/$name" || { echo "[ERROR] Missing release file: $name"; exit 1; }
done
for name in __init__.py xr_contract.py aux_controls.py make_profile.py command_gate.py capture_snapshot.py ready_pose_precheck.py torque_evidence.py build_qp_precision.py build_startup_timing.py \
            sh5_model.py collision_primitives.py check_collision.py hardware.launch.py run_controller.sh \
            run_commissioning_controller.sh log_arms.sh; do
  docker cp "$HERE/$name" "$CONTAINER:$DEST/hardware/$name"
done
docker cp "$HERE/../sim/log_sg2_arm.py" "$CONTAINER:$DEST/sim/log_sg2_arm.py"
echo "[INSTALLED] $CONTAINER:$DEST/hardware (source only; no controller/motor started)"
echo '[SG2 ONCE] Before operating the follower, source ROS/workspace in ai_worker and run:'
echo "  python3 -B $DEST/hardware/build_qp_precision.py"
echo "  python3 -B $DEST/hardware/build_startup_timing.py"
echo '[NEXT] Existing hardware follower in known ready pose, then run_controller.sh sg2|sh5.'
