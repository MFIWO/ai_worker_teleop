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
- [CHANGED 2026-10-28] Controller mode 2 **trigger closing** is now slew-limited
  to 50% of the original configured hand velocity bound (0.5 * 0.85 * URDF
  velocity per joint). The final pose and trigger travel are unchanged.
  Trigger opening, grip-button pinch, and optical modes remain direct.
  This is an output target velocity bound, not a measured 50% motor speed
  calibration; verify the physical closing time before production use.

Select matching `--input` and `--hand-mode` values on PC and robot. Install the
updated sources on both hosts and restart both teleop sessions.

## Latency status

The physical hand latency remains unresolved. A correct or faster retargeting
result does not establish timely motor application or fresh physical feedback.
The launcher uses the **same** Dynamixel port, existing group reads and writes,
and the 30Hz HX5 write overlay. No production SDK or firmware upgrade is used.

[CHANGED 2026-10-28] The existing follower driver now caps the inherited
`ReadMultiDxlData(period_ms)` wait argument to **60ms** (configurable 20–100ms
with `AI_WORKER_HX5_READ_TIMEOUT_CAP_MS`; only the verified follower child sets it).
Previously the measured `ros2_control` loop overrun could become the timeout
for the **next** SDK group read. This bounds that one wait request without
adding serial devices, threads, readers, packet transactions or bypassing any
Dynamixel write/read safety state. Normal short read deadlines are unchanged.

**Limit: this is not full arm/hand ID independence.** SDK `GroupBulkRead::rxPacket`
is all-or-nothing: an arm ID reply omission can invalidate the whole bulk read.
Upstream `DynamixelHardware::write()` skips all writes in `COMM_ERROR`. Therefore
even after this cap, a persistently missing arm ID can stop hand updates.
This patch is a conservative transient-latency mitigation, not proof that
mode 1 will remain responsive on a permanently missing arm ID. Keep the arm
read failure visible; never mark stale hand readings as fresh or force writes
to an unverified bus. The observed ~600ms is not a verified hard-coded runtime
timeout; an idle diagnostic separately uses 600ms. Capture traces to localize
the remaining physical delay.

Before launching the updated robot follower, rebuild its isolated overlay on
the **robot Docker** with motion stopped (the previous overlay remains for
rollback; the versioned output path changes):

```bash
cd /workspace/ai_worker_xr
python3 -m hardware.build_hand_write_rate
python3 -m hardware.build_hand_write_rate --verify-only
# Only if using --hand-trace, also:
python3 -m hardware.build_hand_trace
python3 -m hardware.build_hand_trace --verify-only
```

Use the updated source on both PC and robot, then select
`--input controllers --hand-mode 2` on **both** teleop commands for the
half-rate trigger closure. For stock Cyclo hand input keep
`--input hands --hand-mode 1`. Normal robot bringup selects the bounded
overlay automatically from its manifest. A process using the older library
is rejected instead of silently mixing versions.

A successful C++ overlay build is **not** real-hand certification. First
check follower logs for `BulkRead Rx Fail`, read timeouts and hand servo
movement, then compare ID-response gaps to actual command/stage latency.
Persistent arm-ID loss remains a hard stop and requires a separate,
physically qualified per-ID read-validity and motor-write design.

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
