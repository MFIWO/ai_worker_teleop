# SH5 VR teleoperation and continuous Loop recording

Optical hand selection: `--input hands --hand-mode 1` uses the installed Cyclo original
retargeter, including its native solver and output filter (default). `--input hands --hand-mode 2` uses the HX5 DexPilot
implementation from `xr_ai_teleop`, including its 25-point mapping, scale 1.6,
and alpha 1.0. Select the same mode in both PC and robot commands. The arm
input, mapping and controller are unchanged. Controller input retains its
existing mode 1/2 meanings.

Mode 2 sends fresh HAND_MOVE samples independently of BODY events, runs the
latest samples at up to 50 Hz on the robot, and sends private hand targets
through the existing gate. HOLD/fault stops forwarding; enabling never replays
cached hand goals. The original retargeter is not launched in this mode.
Run `hardware/install_pc.sh` and `hardware/install_robot.sh` on their respective
hosts after updating the checkout. The robot installer includes pinned pure
Python DexPilot dependencies without replacing the installed numerical stack.
Models/dependencies are checked before robot bringup. This selects a hand
mapping; it does not change HX5 firmware, motor profiles or packet cadence.

The PC host launcher automatically authorizes local container root with
`xhost +SI:localuser:root` in the current desktop session. Run it from the
logged-in desktop user's terminal; optical hands with Loop require DISPLAY.
No manual xhost command is needed after reboot.

The two commands below run in separate PC terminals. The PC address must match
the Vuer certificate. Open the installed Loop application first for `--loop vr`.
The PC command waits up to 120 seconds for the robot's Zenoh port 7447 before
starting either ROS process. Enter the robot command in the other terminal
while it displays `[WAIT ZENOH]`; either command may be entered first. A timeout
means the robot session/router still needs attention. Ctrl+C cancels the wait.

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash teleop.sh pc --pc-ip 192.0.2.10 --robot-ip 192.0.2.20 --loop vr --input hands --hand-mode 1
```

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash teleop.sh robot --pc-ip 192.0.2.10 --robot-ip 192.0.2.20 --loop vr --input hands --hand-mode 1 --bringup
```

`--bringup` starts a missing Zenoh router and SH5 follower with
`init_position:=false`. The native hand/head/lift/steering initialization
trajectories are skipped. Once stable feedback is available, the lift first moves to the physical URDF
upper bound (0.0m on this SH5), at a peak speed no greater than 0.12m/s. Only
after measured top position and stop are verified do the two seven-joint arms follow the existing xr_tele quintic ready trajectory over five
seconds, to `[0,0,0,-1.57,0,0,0]` per arm. No hand position trajectory is sent
by this ready mover. Native driver initialization and the follower's existing
hand current commands still run; this option does not turn hand torque off.
This is a startup action, not part of HOLD/resume. An existing follower is
reused without motion; add `--ready-pose` to the **robot** command to request
the same lift-then-arms startup sequence explicitly. Stop old initialization executors and VR
controllers first. Motion failure prevents the VR controller from starting.
Without `--bringup`, router and follower must already run.

The robot foreground session owns the controller and optional Loop streamer.
Closing the PC/VR session stops only Vuer and its keyboard operator. The command
gate holds after 1.5s without a PC intent lease; the robot session, follower, and Loop
streamer remain running. Reconnect with the same PC command, reload Quest, and
explicitly start/resume after fresh tracking and feedback are ready. The
reconnection must first send a disarmed intent; delayed enabled packets cannot
auto-resume motion. An existing unrelated fault still needs the normal reset.

Closing the **robot** terminal first stops command production. After a minimum
three-second exit grace, the SH5 follower stops and a dedicated helper disables
TableSync and sends Torque Enable(64)=0 to each of the 40 finger motors using
direct motor access. It reads back each torque register; only 40 verified zeros
produce `[HAND TORQUE OFF] Verified 40/40 finger motors`. The delay starts on
robot-session exit detection; robot host-lease detection, launch cleanup
and register checks add time. No position/open-hand trajectory is sent.

Tagged launch children are cleaned up even if the launch parent has already
exited. A reused **verified SH5 follower launch** is also stopped at exit so
it cannot own the port during the torque release. A follower without a unique
SH5 launch parent is rejected at startup; stop it and use `--bringup`. Reused
external Zenoh services remain running. New PID files and logs are under
`/workspace/ai_worker_xr_sessions/<id>/`. The wrapper never starts a physical
leader. `--loop off` is the default and disables recording integration. Input mode is selected separately; `--input hands` remains the default for older commands.
`--loop-no-camera` on the robot command selects joint/tactile/status streaming only.
`--dry-run` prints the host command without SSH, ROS, or hardware startup.
The normal host wrapper sends a stdin lease through non-TTY SSH/docker exec.
Ctrl+C closes the lease; closing/killing the host terminal or a lost connection
also ends/expires that terminal's session on the container side. A PC host-lease
expiry ends only the PC session; it does not close the robot session. PC intent
timeouts, B+Y/V HOLD, and Quest tracking loss do **not** trigger torque OFF.
Terminal disconnect logs are saved to the session's `exit.log` (PC side:
`/tmp/sh5_pc_session_exit.log`). Allow cleanup to finish before restarting.

The helper verifies that no other process owns the serial device. It refuses
a second bus connection and unknown SH5 motor mappings. It verifies TableSync
OFF before direct motor access and reads back all 40 Torque Enable registers.
Torque OFF is permitted in both board Operating Modes. Before starting a new
follower, `hand_transport` verifies all 40 torques and TableSync are OFF, sets
HX5 Operating Mode(33) from Preset(1) to Direct Control(0) only when needed,
and reads it back. The old values and verification results are saved in the
session's `hand_transport.json`. It does not change profiles or send goals.
It never clears errors, enables torque or changes operating mode. Failure
is explicitly logged as `[HAND TORQUE OFF FAILED]`; absence of a success log
is not proof of release. Container/power loss or killing the cleanup process
itself cannot execute software cleanup. Restart the complete robot command to
reactivate the follower after a terminal exit; ordinary HOLD/resume remains fast.
The installer adds `pyserial==3.5` only to the VR helper's `vendor` directory if
needed, using the bundled offline wheel; robot internet is not required. Native
driver/underlay sources are not patched.

The local Docker wrapper uses `sg docker` if the account belongs to the Docker
group but its current shell lacks that group. It does not change socket modes.

## Quest controller mode

Pass `--input controllers` on **both** commands. Controller wrists use the
existing hardware arm mapping; Body tracking and fresh robot feedback remain
required. X+A starts/resumes and Y+B requests HOLD. The original SG2 joystick
axis mapping, deadzones and both-stick-click mode switch are reused: right
stick controls lift, left stick controls head or mobile motion. No Loop motion
key grabber runs in this mode; Loop retains its native recording/review keys.

The arm hardware heartbeat now admits auxiliary commands for both SG2 and the
explicit SH5 controller input with a measured auxiliary profile. An old
SG2-only serialization condition previously dropped SH5 stick targets even
though their local calculation ran. Mode selection, calculation and heartbeat
now share the same ACTIVE/tracking/feedback/profile checks. When a stick is
moved, `[STICK INTENT]` reports the PC target being sent, while `[STICK BLOCKED]`
reports missing activation, tracking, feedback or profile. These logs are
rate-limited and do not claim measured motor motion. Both-stick-click mode
switching also requires ACTIVE, matching the existing control path.

### Hand mode 1: original trigger/grip

`--hand-mode 1` is the default. Each side's trigger continuously interpolates
from open to the original grasp (0..1). Grip retains the existing proportional
thumb/index/middle tripod pinch, with ring/little open, and has priority over
trigger. Face buttons retain the existing AX/BY session chords.

### Hand mode 2: finger presets and independent arm grips

Use `--input controllers --hand-mode 2` on both sessions.

| Input | Behavior |
|---|---|
| X | Existing thumb/index/middle grasp path |
| Y | Thumb/index pinch; middle/ring/little stay open |
| A | Restored historical thumb-first four-finger grasp (thumb-out target, then four fingers close with trigger) |
| B | Existing supervised arm ready recovery |
| Left grip | Left arm follows while held; release holds, repress reanchors that side |
| Right grip | Same independent clutch for the right arm; no global recalibration |
| A+X / B+Y | Global start/resume / HOLD |

A starts by moving the thumb toward its historical opposition target while preserving the other four fingers' measured poses until the thumb arrives; only then may trigger closure proceed. The existing 8-second preparation timeout still fails closed if measured thumb arrival never occurs. This restores the old A pose but does not prove four fingertip contacts or self-collision clearance.
Closing and opening are velocity limited, including abrupt trigger release.
Base/lift motion no longer disables gripped arm inputs. Elbow task weight is
zero both stationary and moving; collision and measured tracking checks remain.
B recovery retains the arms-only action and does not raise lift.

SH5 right-stick functions are exclusive: HEAD mode controls lift with vertical
stick travel; MOBILE mode controls base yaw with horizontal travel and ignores
lift input. Both stick clicks toggle modes. The existing left-stick head/base
mapping and activation checks remain in effect.

The final 40 finger targets pass through the robot gate at 100Hz, capped at
85% of each physical URDF velocity limit. Mode 1 and optical hand input retain
the same cap. This reduces speed modestly; it does not reduce holding torque
or prove thermal protection. HOLD retains measured positions rather than
disabling torque. Stale controller input stops updates.


**[INTEGRATED PR #4 — trigger rate]** Controller hand mode 2 trigger **closing** is additionally limited to 50% of the configured HX5 finger velocity setting (while retaining the previous 85% URDF cap). Trigger opening is not deliberately slowed. This is independent of the 30 Hz Dynamixel packet write cadence and does not remove the arm/hand shared BulkRead limitation.

Optical `--input hands --hand-mode 1` delegates landmark-to-joint processing to
Cyclo's installed `scripts.teleop_retargeting` without thumb overrides, filter
changes or solver-budget changes. This is the Cyclo original comparison path.
The existing PC input adapter still publishes each HAND_MOVE immediately with
wrist-local coordinates; BODY_MOVE does not replay hand samples.
Mode 2 fixes the two thumb bends inside the DexPilot kinematic solve while
optimizing the remaining 18 joints, preserving root opposition/pinch solving,
with 14 evaluations / 6ms NLopt budgets (not a hard runtime guarantee).
The robot-side gate/arm and controller-input paths otherwise remain unchanged.
See HX5_LATENCY.md for measured limits and the rejected bypass experiment.

The PC's 30 Hz goal rate does not set the motor bus write rate: the follower
previously resent every hand goal on its 100 Hz hardware write loop. The
`build_hand_write_rate` overlay keeps the same driver and serial ownership,
but sends HX5 model 260/261 goal packets at 30 Hz using the latest command
buffers. Arm writes and all reads retain their existing loop. There is no
additional relay, interpolation, FIFO or motor profile/torque change.
Only the robot session's follower child selects this verified driver build.
`[HX5 WRITE]` reports each board's successful packet rate every five seconds.
Restart an existing follower to select it. This change requires a physical
before/after check; it is not evidence that the remaining board-side plateau
has been resolved. The local `xr_tele` RH5DG2 reference uses a separate hand
command thread and state-receive thread, normally at a 20 Hz command rate;
its input/optimizer code is not substituted for the stock SH5 retargeter.
#### SH5 controller head/mobile refinement (2026-10-28)

[CHANGED] This applies to `--input controllers` with either hand mode 1 or 2;
it does not change optical hands, SG2 mode, arm QP or hardware follower safety.
The **physical, live SH5 URDF** limits remain authoritative: head_joint1
(tilt) = [-0.2317, +0.6951] rad, head_joint2 (pan) = [-0.35, +0.35] rad,
lift = [-0.5, 0] m. Head jog now uses 0.30 rad/s independent of WebXR event
rate (rather than 0.06 rad per input event) and clamps at the *live* URDF
limits without creating an out-of-range command. Lift jog scale is reduced
from 0.005 to 0.002 m per 30 Hz update (approximately 0.06 m/s).

[CHANGED] In MOBILE, pressing either stick immediately commands zero and
disarms translation. Release both sticks and keep them at their physical
centers for at least 0.30 seconds before motion is accepted. The offset
is learned per axis, with at most 0.22 normalized stick displacement;
a larger resting offset blocks motion and must be resolved at the device
or XR input calibration, not by increasing deadzone without limit. When
motion is already at zero, very small center drift is tracked slowly.
The `[STICK INTENT]` log prints `mobile_zero`, `mobile_ready` and
`filter`. Left stick selects **forward/back OR left/right**, never both.
Yaw from the right stick is also suppressed while translation is active.
[CHANGED 2026-10-28] The MOBILE setpoint is now jerk- and
acceleration-clamped: discrete acceleration is `(v_k - v_(k-1))/dt`,
and its step is limited by the declared per-axis `JERK` limit.
During braking, each component approaches zero monotonically and is
clamped at zero rather than crossing to the opposite sign. Reversing
direction intentionally requires the stick to pass through neutral
**after the previous commanded velocity reaches zero**, then a new
opposite press. This is deliberately stricter than the old 0.16-second
reversal timer, which could allow a long stick rebound to reverse.
Yaw and translation also cannot overlap during a mode-direction change.
A forced stop or invalid tracking sample can bypass a smooth jerk ramp
to publish zero immediately. Loss of a fresh controller frame or ACTIVE
gating disarms MOBILE and commands zero.

A base moving *physically* backward while `/cmd_vel.linear.x` stays
zero is **not** joystick drift: inspect `/odom`, swerve drive/controller
logs, drive motor velocity and their timestamps instead. The upstream
ROBOTIS `SwerveDriveController` declares 0.1m/s linear and 0.1rad/s
angular velocity deadbands by default. The official
`ffw_sh5_follower_ai_hardware_controller.yaml` sets
`enabled_speed_limits: false`, so its listed acceleration/jerk limits
are inactive. Together these can make the downstream wheel command
abrupt near zero even when this PC stick ramp is continuous; check
the *live* controller parameters and actual wheel feedback before
retuning firmware/controller limits. This PR deliberately does not change
wheel/steering hardware parameters. Run the first physical trial in a
clear area with lift stationary; compare raw Vuer `thumbstickValue`,
`[STICK INTENT]`, `/cmd_vel`, and odometry. Neither URDF joint limits nor
collision checks should be bypassed to cure a latching hardware fault;
read `/xr_hardware/gate_status` for its reason.

HX5-D20 poses in `hx5_controller_poses.json` use 20 joints per side. The G1
7-joint preset-selection structure and mirrored-hand convention in the local
GR00T checkout (`b042411`, `decoupled_wbc/control/teleop/solver/hand/g1_gripper_ik_solver.py`)
were consulted. The ABXY mapping is the operator's final mapping above; this
checkout's Pico streamer uses trigger/grip for its hand gestures. HX5 angles
are model-based candidates and have not been physically calibrated for contact
or force. No tactile force regulation is added.

Mode 1 retains the 85% physical URDF velocity cap. Mode 2 uses direct position
forwarding; optical hands use the original ROBOTIS hand pipeline directly.
No follower/motor profile, current, or torque settings are changed by that routing.
HOLD retains measured positions rather than disabling torque. Whole-session
tracking, lease, fault, and shutdown handling remain active. Removing the
hand-only age check does not disable these session checks.

For the prior optical hand/Loop keyboard workflow, use `--input hands` on both
commands. That mode retains V start/HOLD/resume and the C review-key mapping.

## Recording behavior

In Loop Preview, select ONE Robot State source, **aiworker** (key:
robotis-vr-sh5). This schema now contains BOTH robot and HX5-D20 hand qpos.
The independent **HX5-D20** source (key robotis-vr-sh5-tactile) has kind
**Tactile**, not Robot State. Retire the old robotis-vr-sh5-hand Robot State
source and re-save the Loop cell configuration: the aiworker key is
unchanged but its channel layout has changed.

Robot State uses robot type ffw_sh5_vr, action space joint_position, and
finger type HX5-D20. The default **training** profile selects robot and
hand joint observations/actions, both hands' numeric pressure/status, and
teleop.flags, teleop.action_valid and teleop.observation_valid (19 channels).
On the robot command, use --loop-channels qpos|training|all or an exact
comma-separated key list. --loop-no-tactile omits both tactile formats;
--loop-no-camera is separate. See hardware/LOOP_SOURCE_SPLIT.md.
Select camera sources exposed by the existing robot camera implementation.
Start,
finish, save and discard the episode in Loop as usual. The wrapper never
invokes Loop's recording start/stop/save/delete API.

The SDK's `on_select` callback confirms these recording metadata values when
Preview opens the source. Unsupported robot/hand/action identities are rejected;
this callback does not initialize, move or reconfigure the physical robot.

**Grip release, B+Y, HOLD and V keep the same Loop recording running.**
Observations, pressure, camera frames and status continue at the same cadence.
For recording only, a missing/non-valid joint action is the current measured
hold position with action_valid=0; mobile hold is a zero velocity. No motion
command is generated by the recorder. A valid fresh gated action is preserved.
Missing/stale observations use fixed-size NaN vectors and observation_valid=0.
This pins every numeric column from the first frame. In the observed Loop
failure, action.lift.joint_position first appeared partway through an episode
and aborted Parquet writing with `parquet columns changed`; it is now present
from the beginning, including when the robot starts in HOLD.

| Group | observation | action |
|---|---|---|
| left_arm / right_arm | measured 7 joints each | final gated 7-joint trajectory |
| left_hand / right_hand | measured 20 joints each | final gated 20-joint trajectory (native retarget or controller grasp) |
| head | measured 2 joints | gated keyboard/controller trajectory |
| lift | measured 1 joint | gated keyboard/controller trajectory |
| mobile | measured odometry vx, vy, wz | gated cmd_vel vx, vy, wz |

### HX5-D20 fingertip tactile

The read-only adapter subscribes to `/left_hand/finger_pressures` and
`/right_hand/finger_pressures` (`robotis_interfaces/msg/HandPressures`). It
uses the existing follower pressure broadcasters and never opens a second
serial connection or starts a tactile grasp controller. For `--loop vr`, the
robot session builds the vendored, unmodified ROBOTIS pressure broadcaster in
`/workspace/ai_worker_xr_hand_pressure` and adds its overlay to the SH5 follower.
It activates `pressure_l_broadcaster` and `pressure_r_broadcaster` on the
existing `/controller_manager`, verifies their 45 state interfaces per hand
and checks both `HandPressures` messages before the ready motion. Already
active matching broadcasters are reused. A follower started without this
overlay must be restarted with `--bringup`. `--loop-no-tactile` skips this step.

Do not additionally run `ros2 launch robotis_hand_bringup
hx5_d20_right.launch.py`: that standalone launch starts another hardware
controller manager and hand initialization/current commands. It is not
necessary to power or enable tactile sensors on the SH5. The isolated plugin
build and state-only broadcasters do not send motor commands.

Each hand's 20-joint observation/action vectors are part of **aiworker**
Robot State. The independent **HX5-D20 Tactile** source carries two typed
channels, left_hand and right_hand, each with up to 45 raw uint8 readings.
The aiworker Robot State additionally declares numeric
`ffw_sh5_vr.observation.left_hand.tactile_pressure` and
`ffw_sh5_vr.observation.right_hand.tactile_pressure` (45 values each) so pressure
appears beside joint positions in `robot-aiworker.parquet`. Per-hand
`tactile_status` is [received,valid,fresh,hz]. These are included by default
under training; qpos deliberately selects only joint observations.

Pressure ordering is `finger_l_sensor1..5` / `finger_r_sensor1..5`, each with
`Present Pressure 1..9` in row-major order (top to bottom, left to right).
The adapter resolves both sensor names and pressure names; array arrival order
does not change this layout. The ROBOTIS finger ordering is thumb, index,
middle, ring, little; pressing the physical right thumb confirmed sensor1
(ID115) changes through 0..255 and returns to zero when released. The two
typed channels represent the two hands, not two individual fingers.
Missing/duplicate sensors, invalid
interface names and values outside 0..255 invalidate that hand's sample.
Malformed or older-than-0.5-second pressures are omitted from typed Tactile;
their numeric mirror is a 45-element NaN vector with fresh=0, never fabricated
zeros. All-zero fresh pressures are valid unloaded sensor readings.
ROS fixed uint8 arrays contain `numpy.uint8` scalars; the parser accepts these
integer readings and converts them to Python integers for the Loop payload.

The HX5-D20 typed Tactile source uses the same 30 Hz PC-anchored clock
and tick sequence as aiworker. Upstream frames faster than 30 Hz are not
all preserved. Raw pressure is not calibrated Newton force. No baseline
subtraction or RH5DG2 force/proximity/palm conversion is applied.

After deployment, with the follower already running, these commands only read
the active controllers/topics inside `ai_worker` (source ROS/workspace first,
with `ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp`):

```bash
ros2 control list_controllers
ros2 topic list -t | grep -E 'finger_pressures|HandPressures'
ros2 topic echo /left_hand/finger_pressures --once
ros2 topic echo /right_hand/finger_pressures --once
```

Successful reception prints `[HX5-D20] left/right: valid 5x3x3 raw tactile
received`. Missing message definitions or stale topics print diagnostics and
leave pressure absent while other VR/Loop channels continue. Do not start a
second hand bringup to obtain tactile while the SH5 follower owns the hardware.

Keys use `ffw_sh5_vr.observation.<group>.joint_position` and
`ffw_sh5_vr.action.<group>.joint_position`; mobile ends with `.velocity`.
All joint angles are radians; lift is meters. Inputs are mapped by joint name,
not by whichever order a message happens to use. A missing/incomplete group is
`None`. Latest action values may remain present during HOLD, accompanied by
the explicit validity/age markers below.

`ffw_sh5_vr.teleop.*` channels are available when selected (the default training profile keeps only flags and action_valid; qpos omits all teleop fields; all keeps every field):

| Channel | Vector entries |
|---|---|
| flags | active, paused, manual_active, fault, status_fresh |
| phase | WAIT_CALIBRATION=0, ALIGN_REQUIRED=1, SETTLING=2, WAIT_START=3, RESUME_SEED=4, ACTIVE=5, HOLD=6, FAULT=7; unknown=-1 |
| action_fresh | left_arm, right_arm, left_hand, right_hand, head, lift, mobile |
| action_valid | same group order; arms/hands require ACTIVE, auxiliaries allow manual_active |
| ages | joint receipt, gate receipt, PC intent receipt in seconds; missing=-1 |
| clock | age of PC clock anchor, robot-minus-PC wall-clock difference in seconds |
| controller_inputs | left trigger, left grip, right trigger, right grip (0..1); missing for optical/stale input; retained during HOLD |
| hand_pose_state | mode (1/2), pending selection, left active pose, right active pose; 0=none, 1=full grip, 2=angled grip, 3=point, 4=thumb-out grip, 5=grip pinch (active sides only) |

Recording freshness is 0.5s and is independent of the longer teleop grace
period. Status sequence numbers reject repeated/out-of-order heartbeats. HOLD
does not stop or re-declare the source and does not close camera streams.

The SDK normally stamps robot wall time, which differs from this PC's clock.
This VR adapter instead uses the SDK's public `SourceProducer.send_robot`
timestamp argument with PC epoch received in the VR heartbeat and robot-local
monotonic elapsed time. Neither system clock is changed. This is a recording
timeline alignment, not measurement of one-way network latency or a guarantee
of hardware capture synchronization. Source receipt ages are recorded.

For a later training export, filter by the relevant group's action_valid and
the desired active/manual policy. Apply any removed time ranges to **all**
streams, including cameras, with one consistent remapping of time. Do not
delete only robot rows. Removing every paused range also removes keyboard
repositioning demonstrations; preserve manual_active when those actions matter.
The original Loop episode remains the full, auditable recording. An exporter
that edits the existing episodes is not run by these commands.

## Keyboard in the Loop window (`--input hands` only)

| Key | Behavior |
|---|---|
| W / S | base forward / backward |
| A / D | base left / right translation |
| O / P | yaw left / right |
| arrows up / down | head tilt up / down |
| arrows left / right | head pan left / right |
| U / I | lift up / down |
| V | start / HOLD / resume arms and hands; clear current motion keys; recording continues |
| C | on Loop review screen: tap = Save Failed; hold = discard, preserving Loop's original C behavior |
| Shift+A or Loop's button | start recording; plain A belongs to base translation in VR mode |

This mode uses X11 passive grabs only on the Loop window. Motion is disabled
outside that window and on focus loss, modifiers, input expiry or missing
measured state. Release keys before refocusing. Operate in the Recorder view;
disable the operator session before typing configuration text with these keys.
While this bridge runs, plain V is consumed before Loop receives it. C forwards
Loop's original C press/release, including its hold duration. Loop's compiled UI
still labels these review actions C / Hold C; use C / Hold C in this VR mode.
The on-screen buttons retain their normal behavior. No recording API is invoked.
V starts or resumes only with fresh bilateral hand/body tracking and a valid
robot profile. It reanchors to measured wrists and uses the existing startup
checks; rejected starts are logged, never queued for later automatic activation.
The bridge owns no follower publishers; all motion passes through the existing
hardware gate, measured-state checks, physical limits and a 0.35s manual lease.
Keyboard speeds are 0.15m/s translation, 0.3rad/s yaw and 0.25rad/s head,
additionally bounded by the physical URDF and measured lead limits. U/I follows
the installed physical leader's full-stick lift jog: measured position +10mm /
-10mm at 100Hz, with physical position and gate slew limits. This is a target
offset, not a fixed lift speed; the former separate 0.04m/s integration is gone.

Loop keyboard mode uses a 63-scalar-joint measured model: 14 arms + lift +
40 fingers + 2 head + 6 wheels. The arm QP fixes finger/head/wheel velocities
to zero and refreshes their positions from measured feedback. Continuous wheel
drive angles are represented modulo 2pi as scalar revolute coordinates; link
poses match their original continuous rotations. These coordinates generate no
wheel commands. Existing 55/57-coordinate profiles remain loadable, but moving
the base and resuming requires the updated live-wheel profile.

V holds/reanchors/resumes within the existing controller process. There is no
recapture service, disk snapshot, collision model regeneration or process
restart on V or after base movement. Release motion keys, then press V once.
A fresh disarmed gate acknowledgement and at least 0.1s after the last base-key
input are required; a rejected V is not queued for automatic later activation.
The model and passive initial checks are prepared once when the robot session
starts. Existing continuous native collision constraints, measured FK and
physical joint bounds apply during operation. Head/lift keyboard targets use
the existing auxiliary collision guard.

SH5 startup has zero extra settling/activation delay and a 0.03s native ramp.
The PC sends stationary measured seed/reactivate immediately from V and waits
for both arm outputs plus the same 0.03s ramp before tracking human motion.
At a healthy 30Hz input/goal cadence the target V-to-ready time is about 0.1s;
this is a configured target, not a measured end-to-end or motor-response
latency guarantee. Missing tracking/feedback, a fault, or preparing a displaced
reference seed can take longer or reject the start. V HOLD keeps recording.

Controller parameter queries retry after 3s without a reply. A changed gate
instance cancels old queries, invalidates the old profile and accepts only
matching current-query responses. This prevents an unanswered pre-restart
query from permanently blocking V with HARDWARE_PROFILE_REQUIRED.

## Arm cadence and the SG2 / hand paths

The old SG2 controller path has 15 controller coordinates. SH5 hands add 40
measured finger coordinates (55 total); keyboard or controller-stick control adds 2 measured head
and 6 measured wheel coordinates (63 total).
SH5 arm IK evaluates over 2,200 collision pairs. Native hand retargeting maps
landmarks directly to finger trajectories and does not solve this arm QP.
Both optical hand landmarks and arm goals use best-effort, keep-last-1 streams;
the arm path additionally applies wrist/workspace guards, native QP and the gate.
The configured WebXR stream and Cartesian goal cadence remain 30 Hz. A faster
finger response alone does not measure the arm solver or network latency.

On this robot, an isolated domain-232 replay of the same stored 55-coordinate
measured pose and 2,248 collision pairs gave:

| Arm executable | Observed output Hz | Median interval | 95th percentile interval |
|---|---:|---:|---:|
| Previous dense QP | 17.42 | 57.93 ms | 60.55 ms |
| Final sparse QP, 55 coordinates / 2,248 pairs | 100.00 | 10.00 ms | 10.08 ms |
| Final sparse QP, 57 coordinates / 2,251 pairs | 100.00 | 10.00 ms | 10.06 ms |

The SH5-only build assembles the same tracking objective, joint bounds and
native collision CBF/slack penalties in sparse form. Every collision distance
is evaluated every cycle. Outside the original collision buffer (5 cm in the
installed config), the original QP gives a row no finite bounds and its slack
has optimum zero. Those unbounded rows and their unused slack variables need
not be assembled. Distance gradients are computed only for bounded rows.
No enabled collision pair is removed from the model. Solver defaults and the
native 2 cm collision target remain unchanged. The SH5 gate now relays at
100 Hz to match the controller; it retains physical velocity slew and checks.

These figures are robot-side synthetic replay measurements, not live VR
end-to-end latency or proof of 100 Hz under every moving posture/camera load.
Optical tracking loss, phase transitions, feedback ages and network jitter can
still cause holds. The Loop adapter marks late tracking/feedback as paused
even when the controller is retaining its last valid goal.

## Installation and scope

Install the helpers with `hardware/install_pc.sh` and
`hardware/install_robot.sh`. Build the updated separate hand executable once
with `python3 -m hardware.build_sh5_hands --rebuild`. The original Cyclo checkout,
underlay executable, physical leader launch and installed Loop package are not
overwritten by the VR option. Camera topics, JPEG paths and rotation helpers
come from the actually installed `ffw_loop_streamer`.

The reference `external_repos/loop` predates the on-robot stereo head JPEG and
driver-side wrist rotation changes. Its whole package must not replace the
installed robot package. Loop application binaries and its recording engine
are not patched; this is an additive source and input bridge.

### Tactile readiness and missing broadcaster fallback

If HandPressures is absent, the Loop adapter reads the named 5x9 pressure
interfaces from `/dynamic_joint_states`. Both paths validate raw integer values
0..255; missing interfaces are not zero-filled. The typed HX5-D20 source is
registered only after both hands supply valid pressure frames, preventing an
empty source from being selected as a recording requirement. Robot frames keep
streaming independently. After source registration, missing/stale hand samples
remain absent. Actual Loop recording must still be verified with the live
follower; no recording API is invoked by this adapter.
