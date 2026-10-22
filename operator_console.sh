#!/usr/bin/env bash
# Start from the local PC desktop terminal, not over an unrelated SSH display.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$script_dir/robotis_vuer/robotis_vuer/start_operator_console.py" "$@"
