#!/usr/bin/env bash
# [ADDED SH5 SIM] Isolated PhysX preparation; does not connect to real robot domain 30.
set -euo pipefail
ROOT="${CYCLO_LAB_ROOT:-$HOME/Downloads/external_repos/cyclo_lab}"
ISAAC="${ISAAC_SIM_ROOT:-$HOME/isaacsim-5.1.0}"
ISAACLAB="${ISAAC_LAB_ROOT:-$HOME/Downloads/IsaacLab}"
[[ -f "$ROOT/scripts/sim2real/bringup/sh5_dds_bringup.py" ]] || { echo 'Missing cyclo_lab SH5 simulation script' >&2; exit 2; }
[[ -f "$ISAAC/python.sh" ]] || { echo 'Missing Isaac Sim 5.1 Python' >&2; exit 2; }
[[ -d "$ISAACLAB/source/isaaclab" ]] || { echo 'Missing Isaac Lab source' >&2; exit 2; }
export ROS_DOMAIN_ID=232
export PYTHONUNBUFFERED=1  # [SH5 SIM] flush readiness/errors to logs
export PYTHONPATH="$ISAACLAB/source/isaaclab:$ISAACLAB/source/isaaclab_assets:$ISAACLAB/source/isaaclab_tasks:$ROOT/source/cyclo_lab:$ROOT/third_party/robotis_dds_python:${PYTHONPATH:-}"
export PYTHONPATH="$HOME/miniconda3/envs/groot-t1/lib/python3.11/site-packages:${PYTHONPATH}" # [SH5 SIM] flatdict dependency already installed in existing env
export PYTHONPATH="$HOME/Downloads/.sg2-bridge-runtime/venv/lib/python3.11/site-packages:${PYTHONPATH}" # [SH5 SIM] existing Cyclone DDS binding
if [[ "${1:-}" == --check ]]; then
  python3 "$(dirname "$0")/sh5_hand_joint_audit.py" \
    --urdf "$HOME/Downloads/external_repos/ai_worker/ffw_description/urdf/ffw_sh5_rev1_follower/ffw_sh5_follower.urdf" \
    --out "$(dirname "$0")/sh5_hand_joint_audit.json"
  echo '[CHECK] isolated ROS_DOMAIN_ID=232; PhysX launch is a separate step'
  exit 0
fi
MODE_ARGS=(--headless)
if [[ "${1:-}" == --gui ]]; then
  # [ADDED SH5 GUI] Only the Isaac viewport mode changes; DDS domain/robot unchanged.
  [[ -n "${DISPLAY:-}" && -S "/tmp/.X11-unix/X${DISPLAY##*:}" ]] || {
    echo '[ERROR] No local X11 display for --gui' >&2; exit 2;
  }
  MODE_ARGS=(--enable_camera_views)
  shift
elif [[ "${1:-}" == --headless ]]; then
  shift
fi
exec "$ISAAC/python.sh" "$ROOT/scripts/sim2real/bringup/sh5_dds_bringup.py" \
  --domain_id 232 "${MODE_ARGS[@]}" --enable_gravity "$@"
