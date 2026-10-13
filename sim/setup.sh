#!/usr/bin/env bash
set -euo pipefail

CYCLO="${CYCLO_LAB_ROOT:-$HOME/Downloads/external_repos/cyclo_lab}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$CYCLO/scripts/sim2real/bringup"

if [ ! -f "$DEST/sh5_dds_bringup.py" ]; then
    echo "[ERROR] Cyclo Lab 설치 경로를 확인하세요: $CYCLO"
    exit 1
fi

# [SAFETY] 기존 파일이 있다면 덮어쓰지 않음
for file in sg2_dds_bringup.py common/sg2_bridge_contract.py; do
    if [ -e "$DEST/$file" ] && ! cmp -s "$HERE/$file" "$DEST/$file"; then
        echo "[ERROR] 기존 파일과 내용이 다릅니다: $DEST/$file"
        exit 1
    fi
done

mkdir -p "$DEST/common"
cp "$HERE/sg2_dds_bringup.py" "$DEST/"
cp "$HERE/common/sg2_bridge_contract.py" "$DEST/common/"

echo "[OK] SG2 DDS Bridge installed"
echo "[INFO] DDS Python dependencies must also be configured."
