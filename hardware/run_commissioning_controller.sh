#!/usr/bin/env bash
# [CHANGED 2026-10-14] Explicit legacy strict policy; normal entry point is run_controller.sh.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HERE/run_controller.sh" "$@" --strict-commissioning
