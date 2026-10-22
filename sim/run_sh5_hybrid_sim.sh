#!/usr/bin/env bash
# Host entrypoint. Copies only this checkout into an isolated container folder.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-help}"
case "$MODE" in
  setup|controller|vuer|smoke|check|start|stop) ;;
  *) echo 'Usage: bash sim/run_sh5_hybrid_sim.sh setup|controller|vuer|smoke|check|start|stop [--server-ip PC_IP]'; exit 0 ;;
esac
shift || true
SERVER_IP=""
while (( $# )); do
  case "$1" in
    --server-ip)
      if (( $# < 2 )) || [[ -z "$2" || "$2" == --* ]]; then
        echo '[ERROR] --server-ip requires the PC IPv4 address'; exit 2
      fi
      SERVER_IP="$2"; shift 2 ;;
    *) echo "[ERROR] Unknown option: $1"; exit 2 ;;
  esac
done
DEST=/workspace/sh5_hybrid_sim_source
CONTAINER=ai_worker
if [[ "$MODE" == vuer || "$MODE" == smoke ]]; then
  CONTAINER=robotis-applications
  DEST=/tmp/sh5_hybrid_sim_source
fi
if [[ "$MODE" == setup || "$MODE" == controller || "$MODE" == vuer || "$MODE" == smoke ]]; then
  docker exec "$CONTAINER" mkdir -p "$DEST"
  for folder in hardware sim robotis_vuer; do
    docker cp "$ROOT/$folder" "$CONTAINER:$DEST/"
  done
fi
TTY=()
if [[ -t 0 && -t 1 ]]; then TTY=(-t); fi
exec docker exec -i "${TTY[@]}" "$CONTAINER" bash "$DEST/sim/sh5_sim_container.sh" "$MODE" "$DEST" "$SERVER_IP"
