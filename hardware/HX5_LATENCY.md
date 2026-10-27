# HX5 hand modes and latency diagnostics

## Operational selection

- Optical mode 1 delegates to the installed Cyclo
  `scripts.teleop_retargeting.main()`. Its native solver and output filter remain.
- Optical mode 2 uses HX5 DexPilot, scale 1.6 and low-pass alpha 1.0.
  Measured thumb MCP/IP bends constrain joints 3/4; the remaining 18 joints
  remain optimization variables. Left/right signs and model bounds are retained.
- Both modes use immediate HAND_MOVE input rather than BODY_MOVE replay.
  The arm controller and global start/HOLD policy remain.
- Controller modes 1/2 retain their trigger/grip and button-selected pose meanings.
  The controller mode-1 velocity scale of 0.85 does not apply to optical hands.

Select matching `--input` and `--hand-mode` values on PC and robot. Install the
updated sources on both hosts and restart both teleop sessions.

## Latency status

The physical hand latency remains unresolved. A correct or faster retargeting
result does not establish timely motor application or fresh physical feedback.
The launcher selects the existing driver's separate 30Hz HX5 write overlay.
Arm writes and the feedback read loop remain unchanged. No production SDK or
palm firmware upgrade is included in this release.

For passive ROS and optional driver-stage capture, use [HAND_TRACE.md](HAND_TRACE.md).
Host write completion does not establish board receipt or motor application.
The board may return stale feedback; physical observation or board-side tracing
is needed to separate that from delayed motion. Binary trace drops must also be
accounted for before comparing stage timings.

## Experimental direct transport

`build_hand_direct.py` and its C++ helper are retained as experimental source.
The candidate did not pass mixed arm/hand communication qualification and is
not connected to `robot_session.py`. Its `follower_environment()` and
`verify_running_follower()` reject operational selection. A successful build or
an offline fake-transport test does not qualify it for physical operation.

Idle probes require an unused serial port and verified torque OFF, and may
temporarily change board mappings/settings before restoring them. They are not
part of normal launch or passive capture. Review their preconditions and scope
before running a separate hardware diagnostic.

Raw experiment logs, board dumps, device-specific reports and private host
metadata are kept locally and excluded from the public source release.
