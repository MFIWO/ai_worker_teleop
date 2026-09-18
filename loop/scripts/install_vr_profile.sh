#!/usr/bin/env bash
# Install recording configuration only. Does not start motors, VR or recording.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
robot_host=${1:-ffw-SNPR48A1112}
ssh -o BatchMode=yes -o ConnectTimeout=8 "$robot_host" \
  'docker exec -i ai_worker python3 -' < "$script_dir/install_vr_profile_in_container.py"
ssh -o BatchMode=yes -o ConnectTimeout=8 "$robot_host" \
  'docker exec ai_worker bash -lc "source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build --symlink-install --packages-select ffw_loop_streamer && source install/setup.bash && ros2 launch ffw_loop_streamer loop_streamer.launch.py --show-args"'
