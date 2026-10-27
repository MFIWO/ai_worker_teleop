# VR robot, hand and tactile sources

`--loop vr` now declares three independent data sources. Camera sources are
unchanged. This code does not start or stop recordings.

| Kind | Registration key | Contents |
| --- | --- | --- |
| Robot State | `robotis-vr-sh5` | `aiworker`: arms, head, lift, mobile, teleop diagnostics |
| Robot State | `robotis-vr-sh5-hand` | Hand joint observations/actions, tactile health/timing, controller inputs |
| Tactile | `robotis-vr-sh5-tactile` | `HX5-D20`: `left_hand`, `right_hand`, 45 raw uint8 samples each |

Pressure uses `SourceProducer.send_tactile`, not robot state channels. The
pressure order is fingers 1 through 5, each with row-major 3 by 3 sensors, as
validated by `hx5_tactile.py`. Missing or stale pressure is omitted. Joint
angles remain Robot State readings; they are not tactile measurements.

All three senders share one PC-anchored microsecond timestamp and tick sequence.
Existing channel names are retained for robot/hand state. The existing robot
source key is retained; its layout no longer includes hand channels. Existing
recording configurations therefore require source/layout compatibility checks
before recording. A display name alone does not identify a stored source.

On 2026-10-27 the user confirmed the tactile name `HX5-D20`. An isolated,
read-only recorder registered all three sources with the running Loop app;
`/preview/sources` confirmed `aiworker` and Hand as `robot`, and `HX5-D20` as
`tactile`, with the layout above. No test pressure or robot readings were sent
to the live Loop app and no recording was started.

The three changed runtime modules (`loop_clock.py`, `loop_vr_streamer.py`,
`host_lease.py`) were installed in robot container
`/workspace/ai_worker_xr/hardware/`, with numbered backups. SHA-256 hashes
matched the local files. The PC source is already bind-mounted from this
checkout. The original two teleop commands can be used on the next run.

The saved `test_ffw_sg2_hx5` configuration itself could not be inspected:
`/cell-configs` requires authentication (HTTP 401). A matching display name
does not prove that a saved source identity/layout matches the new sender.
In Loop select the available `aiworker`, Hand and `HX5-D20` sources when updating
that configuration. Real sensor reception and an actual saved episode still
require verification during a physical session; no follower was running during
this deployment.

## Stop investigation

The PC container's `/tmp/sh5_pc_session_exit.log` contained
`Host terminal/connection lease ended`. The old message did not distinguish
stdin EOF, explicit Ctrl+C, or a host heartbeat timeout. `host_lease.py` now
logs that distinction without changing the three-second timeout.

Optical tracking freshness is 0.5 seconds; already active sessions may hold
unchanged goals and recover until input age exceeds five seconds. Partial hand
frames do not refresh tracking liveness. Full-frame retarget failures can still
disable the arm session. No limit, collision check or motor error guard has been
relaxed based on the connection-exit message. Actual repeated STOP/FAULT events
from the PC publisher are needed to identify the cause of recurrent motion holds.

## Validation

Run `python3 -m unittest hardware.test_loop_vr hardware.test_loop_split`.
The installed SDK schema/encoding test is skipped where loop-sdk is absent;
run it inside the robot container with its `/opt/venv` site-packages available.
Tests do not connect to Loop, start ROS nodes or command motors.

The robot container passed three SDK/unit tests and one isolated ROS callback
test on localhost/domain 232 with a fake SDK sink. The callback test verifies
continuous HOLD recording, separate robot/hand layouts, shared timestamps,
the typed tactile declaration, and clean shutdown.

The existing `hardware.test_tracking_grace` AST fixture omits the production
`FEEDBACK_MAX_AGE` constant and currently raises `AttributeError`; that suite
cannot validate production behavior without fixing its fixture/expectations.

## Native hand boundary fault (2026-10-27)

User logs repeatedly reported `native hand target outside physical bounds`
within 0.1–0.8 seconds of ACTIVE. The installed native retargeter selects the
HX5-D20 model; its optimizer's `set_joint_limit` expands each limit by 0.001 rad.
The gate previously accepted only 0.0001 rad numerical overshoot, so ordinary
boundary solutions could fault the entire session. The incoming solver margin
is now 0.001001 rad (including floating-point allowance), and accepted targets
are clamped to the unchanged physical URDF bounds before slew limiting/output.
Larger violations and nonfinite targets still latch a fault, now including the
joint name, target and limits. The mismatch is confirmed in code; exact values
of the user's original rejected targets were not recorded, so a physical rerun
is still required to verify that this resolves those incidents.

`hardware.test_native_hand_bounds` covers both hands, both boundary directions,
clamping, gross overshoot, nonfinite data and invalid joint names. No physical
controller, motor or recording is started by these tests.

The retired display name `AI Worker VR / HX5-D20` is no longer registered by
this code. Existing saved cell-config entries can still reference it; remove or
ignore that entry in Loop and select the separated sources. The live source
list was empty after the user's sessions stopped, so no obsolete live source
could be hidden through `/preview/sources/{id}/selection` at inspection time.
