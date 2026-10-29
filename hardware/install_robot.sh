#!/usr/bin/env bash
# [ADDED] Run on the robot HOST with this branch checked out; COPY ONLY, never launches motors.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${AI_WORKER_CONTAINER:-ai_worker}"
DEST=/workspace/ai_worker_xr
docker exec "$CONTAINER" mkdir -p "$DEST/hardware" "$DEST/sim"
# Official state-only plugin, loaded into the existing SH5 controller manager.
for name in build_hand_pressure.py hand_pressure_bringup.py hand_pressure_broadcasters.yaml setup_hand_pressure_interfaces.sh; do
  test -f "$HERE/$name" || { echo "[ERROR] Missing release file: $name"; exit 1; }
done
test -f "$HERE/../vendor/robotis_hand_pressure_broadcaster/package.xml"
for name in HandPressures TactileSensor; do
  test -f "$HERE/../vendor/robotis_tactile_interfaces/msg/$name.msg"
done
# Copy named source files only: no .git, credentials, logs, built overlays or runtime profiles.
for name in __init__.py arm_posture.py base_collision_cover.py xr_contract.py aux_controls.py make_profile.py command_gate.py capture_snapshot.py ready_pose_precheck.py torque_evidence.py build_qp_precision.py build_startup_timing.py \
            sh5_model.py sh5_hands_model.py build_sh5_hands.py collision_primitives.py check_collision.py hardware.launch.py run_controller.sh \
            run_commissioning_controller.sh log_arms.sh log_hands.sh keyboard_controls.py teleop_cli.py session_processes.py \
            robot_session.py run_robot_session.sh ready_pose.py host_lease.py hand_shutdown.py hand_transport.py build_hand_write_rate.py hx5_write_cadence.hpp build_hand_trace.py hx5_stage_trace.hpp hx5_trace_port.hpp hand_trace_capture.py analyse_hand_trace.py hand_capture_runner.py controller_hand_modes.py ready_recovery.py loop_vr_contract.py loop_channels.py loop_vr_streamer.py loop_clock.py hx5_tactile.py sh5_controller_hands.py hx5_controller_poses.json sh5_sparse_qp.cpp xr_hand_input.py xr_ai_hx5_retarget.py xr_hand_retarget_node.py thumb_flexion.py robotis_hand_retarget_node.py cyclo_hand_retarget_node.py hx5_latency_probe.py; do
  test -f "$HERE/$name" || { echo "[ERROR] Missing release file: $name"; exit 1; }
done
for name in __init__.py arm_posture.py base_collision_cover.py xr_contract.py aux_controls.py make_profile.py command_gate.py capture_snapshot.py ready_pose_precheck.py torque_evidence.py build_qp_precision.py build_startup_timing.py \
            sh5_model.py sh5_hands_model.py build_sh5_hands.py collision_primitives.py check_collision.py hardware.launch.py run_controller.sh \
            run_commissioning_controller.sh log_arms.sh log_hands.sh keyboard_controls.py teleop_cli.py session_processes.py \
            robot_session.py run_robot_session.sh ready_pose.py host_lease.py hand_shutdown.py hand_transport.py build_hand_write_rate.py hx5_write_cadence.hpp build_hand_trace.py hx5_stage_trace.hpp hx5_trace_port.hpp hand_trace_capture.py analyse_hand_trace.py hand_capture_runner.py controller_hand_modes.py ready_recovery.py loop_vr_contract.py loop_channels.py loop_vr_streamer.py loop_clock.py hx5_tactile.py sh5_controller_hands.py hx5_controller_poses.json sh5_sparse_qp.cpp xr_hand_input.py xr_ai_hx5_retarget.py xr_hand_retarget_node.py thumb_flexion.py robotis_hand_retarget_node.py cyclo_hand_retarget_node.py hx5_latency_probe.py; do
  docker cp "$HERE/$name" "$CONTAINER:$DEST/hardware/$name"
done
for name in build_hand_pressure.py hand_pressure_bringup.py hand_pressure_broadcasters.yaml setup_hand_pressure_interfaces.sh; do
  docker cp "$HERE/$name" "$CONTAINER:$DEST/hardware/$name"
done
docker exec "$CONTAINER" mkdir -p "$DEST/vendor" "$DEST/hardware/xr_ai_assets"
docker cp "$HERE/xr_ai_assets/." "$CONTAINER:$DEST/hardware/xr_ai_assets/"
docker cp "$HERE/../vendor/dex_retargeting" "$CONTAINER:$DEST/vendor/"
docker cp "$HERE/../vendor/robotis_hand_pressure_broadcaster" "$CONTAINER:$DEST/vendor/"
docker cp "$HERE/../vendor/robotis_tactile_interfaces" "$CONTAINER:$DEST/vendor/"
# Pure Python additions only; retain installed torch, numpy, Pinocchio and nlopt.
for wheel in anytree-2.12.1-py3-none-any.whl trimesh-4.4.9-py3-none-any.whl pytransform3d-3.14.4-py3-none-any.whl; do
  docker cp "$HERE/vendor/$wheel" "$CONTAINER:/tmp/$wheel"
  docker exec "$CONTAINER" python3 -m pip install --no-deps --no-index --upgrade --target "$DEST/vendor" "/tmp/$wheel"
done
docker cp "$HERE/../sim/log_sg2_arm.py" "$CONTAINER:$DEST/sim/log_sg2_arm.py"
if ! docker exec "$CONTAINER" python3 -c "import sys; sys.path.insert(0,'$DEST/vendor'); import serial" >/dev/null 2>&1; then
  WHEEL=pyserial-3.5-py2.py3-none-any.whl
  test -f "$HERE/vendor/$WHEEL" || { echo "[ERROR] Missing offline dependency: hardware/vendor/$WHEEL"; exit 1; }
  docker cp "$HERE/vendor/$WHEEL" "$CONTAINER:/tmp/$WHEEL"
  docker exec "$CONTAINER" python3 -m pip install --no-index --target "$DEST/vendor" "/tmp/$WHEEL"
fi
echo "[INSTALLED] $CONTAINER:$DEST/hardware (source only; no controller/motor started)"
echo '[SG2 ONCE] Before operating the follower, source ROS/workspace in ai_worker and run:'
echo "  python3 -B $DEST/hardware/build_qp_precision.py"
echo "  python3 -B $DEST/hardware/build_startup_timing.py"
echo '[NEXT] Existing hardware follower in known ready pose, then run_controller.sh sg2|sh5.'

echo "[SH5 HANDS BUILD] cd $DEST && python3 -m hardware.build_sh5_hands"
echo "[SH5 HAND WRITE BUILD] cd $DEST && python3 -m hardware.build_hand_write_rate"
echo "[SH5 PRESSURE BUILD] cd $DEST && python3 -m hardware.build_hand_pressure (also automatic for --loop vr)"
echo "[SH5 HANDS RUN] bash $DEST/hardware/run_controller.sh sh5 --hands"
