#!/usr/bin/env bash
set -eo pipefail
MODE="$1"
DEST="$2"
SERVER_IP="${3:-}"
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=232 RMW_IMPLEMENTATION=rmw_fastrtps_cpp SH5_SIM_ONLY=1
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS CYCLONEDDS_URI
export PYTHONPATH="$DEST:$DEST/robotis_vuer:/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
cd "$DEST"
case "$MODE" in
  setup) exec python3 -m sim.build_sh5_controller ;;
  controller)
    python3 -m sim.check_sh5_sim --preflight
    python3 -m sim.prepare_sh5_sim
    exec ros2 launch "$DEST/sim/sh5_hybrid_sim.launch.py"
    ;;
  vuer|smoke)
    test -f /root/ros2_ws/cert.pem && test -f /root/ros2_ws/key.pem || {
      echo '[ERROR] Existing Vuer cert.pem/key.pem missing in /root/ros2_ws'; exit 2;
    }
    cp /root/ros2_ws/cert.pem /root/ros2_ws/key.pem "$DEST/robotis_vuer/robotis_vuer/"
    if [[ -n "$SERVER_IP" ]]; then
      python3 -m hardware.vuer_tls --cert /root/ros2_ws/cert.pem \
        --key /root/ros2_ws/key.pem --server-ip "$SERVER_IP"
      echo "[QUEST URL] https://${SERVER_IP}:8012?ws=wss://${SERVER_IP}:8012"
    fi
    if [[ "$MODE" == smoke ]]; then exec python3 -m sim.test_sh5_hybrid_runtime; fi
    exec python3 -m robotis_vuer.vr_publisher_sh5_sim
    ;;
  check) exec python3 -m sim.check_sh5_sim ;;
  start) exec ros2 service call /sh5_sim/enable std_srvs/srv/SetBool '{data: true}' ;;
  stop) exec ros2 service call /sh5_sim/enable std_srvs/srv/SetBool '{data: false}' ;;
esac
