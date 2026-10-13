#!/usr/bin/env bash
# [ADDED] Run INSIDE ai_worker Docker. Original installed controller untouched.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
# [ADDED] Same existing Python dependency path used by the verified launch.
if [[ -d /opt/venv/lib/python3.12/site-packages ]]; then
  export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
test "$ROS_DOMAIN_ID" = 231 || { echo '[ERROR] This launcher is simulation domain231 only'; exit 1; }
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
OVERLAY=/workspace/sg2_extension_overlay
PROFILE=/workspace/sg2_xr_profiles/extension
test -f "$OVERLAY/install/local_setup.bash" || { echo '[ERROR] Build extension precision overlay first'; exit 1; }
test -f "$PROFILE/sg2_near_extension.urdf" -a -f "$PROFILE/near_extension.yaml" || { echo '[ERROR] Generate extension profile first'; exit 1; }
source "$OVERLAY/install/local_setup.bash"
BIN="$(ros2 pkg prefix cyclo_motion_controller_ros)/lib/cyclo_motion_controller_ros/vr_controller_node"
if ! ldd "$BIN" | grep -E 'cyclo_motion_controller_core.*=> /workspace/sg2_extension_overlay/'; then
  echo '[ERROR] Runtime did not select precision overlay core library'; exit 1
fi
export CYCLO_SG2_QP_PRECISION=1
echo '[EXTENSION] QP numeric precision overlay ON (1e-5), joint4 upper=-.45, original safety checks retained.'
echo '[EXTENSION] Start new fixed-calibration extension publisher; wait for XR RESUME READY.'
exec ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
  controller_type:=vr hand:=false arm:=false \
  config_file:="$PROFILE/near_extension.yaml" \
  follower_urdf_path:="$PROFILE/sg2_near_extension.urdf"
