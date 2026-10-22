#!/usr/bin/env bash
# [ADDED 2026-10-14] PC HOST: export the already NTP-synchronised PC clock to one robot.
# chronyd -x NEVER adjusts the PC clock; the existing host timesync service owns it.
set -euo pipefail
ROBOT_IP="${1:-192.168.5.102}"
CONTAINER="${AI_WORKER_PC_CONTAINER:-robotis-applications}"
[[ "$ROBOT_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo '[ERROR] IPv4 robot address required'; exit 2; }
[[ "$(timedatectl show -p NTPSynchronized --value)" == yes ]] || {
  echo '[ERROR] PC host clock is not NTP-synchronised; no LAN time server started'; exit 1;
}
PC_IP="$(ip -4 route get "$ROBOT_IP" | awk '{for(i=1;i<=NF;i++)if($i=="src"){print $(i+1);exit}}')"
[[ -n "$PC_IP" ]] || { echo '[ERROR] No local route to robot'; exit 1; }
[[ "$(docker inspect "$CONTAINER" --format '{{.HostConfig.NetworkMode}}')" == host ]] || {
  echo '[ERROR] PC time server requires the existing host-network container'; exit 1;
}
docker exec -e TIME_PC_IP="$PC_IP" -e TIME_ROBOT_IP="$ROBOT_IP" "$CONTAINER" bash -c '
set -euo pipefail
command -v chronyd >/dev/null || { echo "[ERROR] Install chrony inside the PC container first"; exit 1; }
install -d -m 0750 -o _chrony -g _chrony /run/ai_worker_time
CONF=/run/ai_worker_time/chrony.conf
if [[ -s /run/ai_worker_time/chronyd.pid ]]; then
  PID=$(cat /run/ai_worker_time/chronyd.pid)
  if kill -0 "$PID" 2>/dev/null; then
    grep -q "^bindaddress $TIME_PC_IP$" "$CONF" && grep -q "^allow $TIME_ROBOT_IP/32$" "$CONF" || {
      echo "[ERROR] Existing time server belongs to another interface/robot; inspect before changing"; exit 1;
    }
    echo "[TIME SERVER] Already serving $TIME_PC_IP to $TIME_ROBOT_IP only"
    exit 0
  fi
fi
printf "%s\n" \
  "# Local PC clock is disciplined by host systemd-timesyncd, not this server." \
  "local stratum 10" \
  "allow $TIME_ROBOT_IP/32" \
  "bindaddress $TIME_PC_IP" \
  "cmdport 0" \
  "bindcmdaddress /run/ai_worker_time/chronyd.sock" \
  "pidfile /run/ai_worker_time/chronyd.pid" > "$CONF"
chronyd -x -f "$CONF"
echo "[TIME SERVER] $TIME_PC_IP:123/udp -> authorised client $TIME_ROBOT_IP (-x: no clock adjustment)"
'
