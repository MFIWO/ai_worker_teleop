# HX5 hand modes and latency diagnostics

Current status (2026-10-29): the operator reports the runtime latency resolved.
The final field remedy was not specified in the conversation; do not attribute
that outcome to a particular code patch. The measurements below are historical
diagnostic evidence, not the current operational status.

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

At the time of the following investigation, physical hand latency was unresolved. A correct or faster retargeting
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


## Idle board clock isolation, 2026-10-29

After both teleop sessions exited, the follower port was checked idle, both
TableSync enables were OFF, and all forty finger torques were confirmed OFF.
The probe saved the board fields, zeroed every motor write size before enabling
TableSync, and substituted the first motor read address with Realtime Tick
(120) in each finger table. No motor torque, goal, drive mode, current limit or
profile was written. The measured clock below is the first finger motor clock
mirrored by each board, not host ROS publication frequency.

Results from short repeated idle comparisons (1.5–3 seconds per phase):

| Read traffic alongside both hands | Left board 140 | Right board 110 |
| --- | --- | --- |
| Hands only | about 100 Hz; largest gap 10–11 ms | about 100 Hz; largest gap 10–11 ms |
| Right arm Bulk Read, IDs 1–7 | about 100 Hz | about 10 Hz; largest gap 310 ms |
| Both arms Fast Sync Read | about 9 Hz; largest gap 260 ms | about 12 Hz; largest gap 324 ms |
| Wrist ID7 alone, indirect 698/11 | about 99 Hz | about 21 Hz; largest gap 311 ms |
| Wrist ID37 alone, indirect 698/11 | about 10 Hz; largest gap 321 ms | about 66 Hz; largest gap 72 ms |
| Wrist ID7 model-number read, 0/2 | about 100 Hz | about 14 Hz; largest gap 310 ms |
| Wrist ID37 model-number read, 0/2 | about 11 Hz; largest gap 312 ms | about 80 Hz; largest gap 70 ms |

Other individual arm IDs usually maintained about 100 Hz, with occasional
20–70 ms gaps. Splitting every arm read into individual transactions, adding
0.3/1 ms between them, changing read addresses, shortening replies, and Fast
Sync Read did **not** remove the stalls. These are diagnostic comparisons,
not qualified alternative production transports. One both-arm Bulk Read phase
aborted with SDK receive error -3001; its finally block still restored every
saved field and verified all forty torques OFF. Every probe reported successful
restoration; the robot remains stopped.

The installed model table identifies wrist IDs 7/37 as model 2000
`ph42_020_s300` (firmware 15), arm IDs 6/36 as model 4030
`ym070_210_r099` (firmware 12), and sampled finger IDs 117/147 as model 1710
`xm335_t323` (firmware 53). HX5 board firmware is 10. Do not identify model1710
as XC330 based on its similar control table. Sampled finger drive mode was12,
operating mode5, profile acceleration50, profile velocity100, current limit300;
these values were read only and have not been qualified as a latency fix.

This localizes a repeatable **board-side cached feedback refresh stall**
associated especially with PH42 wrist read traffic. It does not yet prove the
mechanism inside firmware, nor establish that physical goal application has the
same delay. Earlier passive command/reference traces showed roughly 570–610 ms
reference-to-feedback lag, while hardware-to-packed goals took about0.11 ms
and follower reads about3.75 ms median. Shortening a host RX deadline to60 ms
cannot guarantee a fresh value inside an otherwise successful board reply.
No operational driver change is claimed to resolve this finding.

Reproduce the wrist comparison only after stopping both sessions, confirming
all finger torques OFF, and reviewing the reversible probe procedure:

```bash
python3 -B -m hardware.hx5_latency_probe \
  --description <verified-profile>/snapshot/robot_description.urdf \
  --report <local-log-directory>/idle_wrist_clock.json \
  --execute --wrist-isolation
```

Raw local reports retained in `/workspace/ai_worker_xr_logs/`:
`idle_clock_probe_20261029.json`, `idle_clock_bulk_probe_20261029.json`,
`idle_clock_split_probe_20261029.json`,
`idle_clock_addresses_probe_20261029.json`,
`idle_clock_per_motor_20261029.json`, and
`idle_clock_wrist_lengths_20261029.json`.
Next investigation requires the HX5 firmware implementation/update information
and an explanation of the PH42/HX5 bus interaction; the operator is checking
availability. Do not drop wrist feedback IDs, mark cached feedback fresh, or
silently deploy the unqualified direct transport as a workaround.


### Follow-up: board clock, wrist timing, and reversible comparisons

`idle_dual_clock_20261029.json` records the mirrored motor clock together with
board Realtime Tick(100), watchdog(8), TableSync(70), Status(106), and individual
SDK read start/end/comm/error values. During 297–308 ms right-hand cached-clock
holds, the board clock advanced by approximately the same number of
milliseconds. The left side showed corresponding holds while its board clock
also advanced. Wrist reads had median durations of 0.939 ms (ID7) and 0.942 ms
(ID37), maxima 1.40/1.78 ms, and no wrist read errors in that capture. Therefore
neither a stopped board clock nor a 310 ms PH42 response explains those holds.
Watchdog remained0 and TableSync1. Status106 sometimes returned16/32/48;
the current public table marks those bits unused, so their firmware10 meaning
must not be guessed. The capture also contains33 failed board140 read attempts;
read retries and observation overhead are retained in the raw report.

Reversible torque-OFF comparisons:

- PH42 Return Delay values0/25/100/254 (0–508 microseconds) did not eliminate
  cached-clock stalls. The largest value improved some short comparisons but
  still showed 80–126 ms gaps, and another rate comparison retained a304 ms gap.
- Reducing wrist read frequency to50/25/10 Hz did not reliably eliminate stalls.
- Temporarily changing wrist IDs7→8 and37→38 did not eliminate the respective
  board stalls. Both IDs were restored to7/37, verified, with torque OFF.
- With TableSync OFF, a read-only direct motor-clock comparison sampled111/141
  400 times in4 seconds without wrist traffic. After adding ID7 reads, direct
  ID111 reading failed with SDK -3001 after one sample. Thus direct transport
  remains unqualified; disabling TableSync alone is not a demonstrated fix.

All PH42 return-delay settings were restored and verified. One rate comparison
initially failed readback during board restoration at140:1228 and110:1426.
An idle recovery retried the saved settings and verified **every** saved field,
all40 finger torques OFF, both wrist torques OFF, and restored wrist delays.
The report now has `restored:true` and a `restore_recovery` note.
No goal, torque, operating mode or motor profile changes were made.

Additional local reports: `idle_wrist_return_delay_20261029.json`,
`idle_wrist_rate_delay_20261029.json`, `idle_wrist_id_isolation_20261029.json`,
and `idle_direct_clocks_20261029.json` in the same robot log directory.

The operator initially preferred firmware investigation over robot rewiring,
and subsequently asked about a hand-only U2D2 connection to the PC. That is a
supported official diagnostic arrangement. Keep the hand electrically isolated
from the PH42 RS-485 segment for the A/B comparison; maintain its required power.
PC Wizard is already installed at
`/home/kimm/ROBOTIS/DYNAMIXEL Wizard 2.0`. Select the actual U2D2 port,
Protocol2.0 and the existing4 Mbps baud. Do not open the same port from Wizard
and a diagnostic process simultaneously. No firmware has been flashed, and no
operational latency fix has been certified.

Official connection reference:
https://docs.robotis.com/docs/systems/hx5_d20/quick_start_guide/dxl_wizard_guide/


### PC U2D2 hand-only inventory, 2026-10-29

PC `/dev/ttyUSB0`, Protocol2.0,4 Mbps: right HX5 board110,
model260, firmware10; all20 mapped XM335 model1710 motors responded and had
Torque Enable0. Board140, default ID10, and wrist IDs7/37 did not respond.
No port owner was detected before this read-only inventory. No configuration,
TableSync, torque, goal, or firmware writes were performed on the PC connection.

The board reported Hardware Error Status(71)=1 (input-voltage error),
TableSync0, watchdog0, Status106=0. Ten successive voltage reads were
46.25–46.29 V using the public table's0.01 V/count unit. Configured limits
were16–32 V; motor111's regulated-side voltage was11.8–11.9 V. The operator
reports using a U2D2 connection powered from a G1 robot battery, with the hand
on its4-pin connection and USB connected to this PC. Actual connector voltage
has not been independently measured; no claim about an undocumented voltage
conversion stage is established. The USB interface is not a24 V regulator.

Clock/TableSync activation comparison is deferred until the supplied hand
voltage is confirmed compatible and the input-voltage error is understood.
The official hand-only Wizard setup recommends24 V. Do not raise the board's
protection threshold just to suppress this error. Raw inventory and voltage
JSON are preserved under ignored local `hardware/evidence/u2d2_pc_20261029/`.


### 24 V PC baseline completed

After the operator changed the supply, the same right-hand board110 reported
24.32 V, Hardware Error Status0 and Status0. All20 mapped XM335 torques were
OFF. A reversible20-second isolated TableSync clock probe then collected2000
samples. Every finger channel's first motor clock changed at every observation
(approximately100 observed changes/s). With timestamps taken after all channel
reads, median change gap was10.00 ms, p95 10.89 ms, maximum11.27 ms on allfive
channels. No310 ms cache plateau occurred. The original probe's earlier-in-cycle
first-channel timestamp yielded maximum10.25 ms; use the all-channel timestamp
summary for cross-channel comparisons rather than mixing the two definitions.

The capture contained14156 SDK read attempts, zero communication/execution
failures, median0.998 ms, p95 1.103 ms, maximum1.759 ms. Hardware Error Status
remained0 throughout. All saved board fields were restored and verified without
errors; postchecks confirmed TableSync0, input24.32 V, error0 and all20 torques
OFF. No motor goal, torque, profile, drive mode or voltage-limit changes were
made. The probe disabled all motor write mappings before enabling TableSync.

Local evidence in `hardware/evidence/u2d2_pc_20261029/`:
`inventory_after_24v.json`, `clock_baseline_24v.json`, and
`clock_baseline_24v_summary.json`. The generic probe now accepts an optional
`phases` list so a hand-only bus does not query absent arm motors; a fault-
injection restoration test covers that path (four diagnostic tests passed).

This establishes a responsive right-hand cache under24 V on an isolated PC
bus. It does not measure physical goal application, left-hand behavior or
command-to-motion latency. Supply and bus composition differ from the robot
configuration, so this comparison alone cannot assign causality to either
change. A shared-bus comparison at a compatible hand supply, or verified
firmware changes followed by mixed-bus qualification, remains necessary before
claiming the original teleop latency resolved. Do not join independent supply
outputs when arranging a mixed-bus comparison.
