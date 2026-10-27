#!/usr/bin/env bash
# Run on the desktop host. The authorization lasts for this X session.
set -euo pipefail
if [[ -z "${DISPLAY:-}" ]]; then
  if [[ "${1:-}" == --required ]]; then
    echo '[X11 ERROR] DISPLAY is missing. Run teleop.sh pc from a terminal in the PC desktop session.' >&2
    exit 1
  fi
  exit 0
fi
if ! command -v xhost >/dev/null 2>&1; then
  echo '[X11 ERROR] Install x11-xserver-utils on the PC host, then retry.' >&2
  exit 1
fi
if ! xhost +SI:localuser:root >/dev/null; then
  echo '[X11 ERROR] Cannot authorize the container. Run from the logged-in desktop user with its DISPLAY and XAUTHORITY.' >&2
  exit 1
fi
echo '[X11] Local container root authorized for this desktop session.'
