#!/usr/bin/env bash
# [CHANGED 2026-10-14] Explicit legacy strict policy; normal entry point is run_xr_hardware_pc.sh.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HERE/run_xr_hardware_pc.sh" "$@" --strict-commissioning
