# SH5 VR teleoperation and continuous Loop recording

The two commands below run in separate PC terminals. The PC address must match
the Vuer certificate. Open the installed Loop application first for `--loop vr`.
The PC command waits up to 120 seconds for the robot's Zenoh port 7447 before
starting either ROS process. Enter the robot command in the other terminal
while it displays `[WAIT ZENOH]`; either command may be entered first. A timeout
means the robot session/router still needs attention. Ctrl+C cancels the wait.

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash teleop.sh pc --pc-ip 192.168.5.2 --robot-ip 192.168.5.101 --loop vr
```

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash teleop.sh robot --pc-ip 192.168.5.2 --robot-ip 192.168.5.101 --loop vr --bringup
```

`--bringup` starts a missing Zenoh router and native SH5 follower. The native
follower performs its existing real initialization motions, including arms,
fingers, head, lift and steering. Existing services are reused; a mismatched
description, duplicate controller or missing feedback rejects startup. After a
new follower starts, the session waits for stable measured ready arms and for
the native trajectory executors to finish. An existing follower is never moved
to ready by this wrapper. Without `--bringup`, both services must already run.

The foreground session owns the VR controller and optional Loop streamer. On
Ctrl+C those processes stop, followed by the router/follower started by this
session. Tagged launch children are cleaned up even if the launch parent has
already exited, including hand initialization executors. Already-running
external services reused by this command are not owned by it. New PID files and logs are under
`/workspace/ai_worker_xr_sessions/<id>/`. The wrapper never starts a physical
leader. `--loop off` is the default and starts only the optical hand/arm path.
`--loop-no-camera` on the robot command selects joint/tactile/status streaming only.
`--dry-run` prints the host command without SSH, ROS, or hardware startup.
Allow the robot terminal to finish its `[STOP]` messages before closing it.

The local Docker wrapper uses `sg docker` if the account belongs to the Docker
group but its current shell lacks that group. It does not change socket modes.

## Recording behavior

In Loop Preview, select **AI Worker VR / HX5-D20** (stable source key
**robotis-vr-sh5**), robot type **ffw_sh5_vr**, action space **joint_position**,
and finger type **HX5-D20**. This one robot source carries the arm/hand angles,
hand commands, raw fingertip pressures and teleop metadata together. The
finger-type selector is advertised through the same SDK `finger_type` axis
used by xr_tele for RH5DG2. Reopen/reselect a saved Preview configuration after
updating to pick up the new option and channels. Select the
camera sources exposed by the existing robot camera implementation. Start,
finish, save and discard the episode in Loop as usual. The wrapper never
invokes Loop's recording start/stop/save/delete API.

The SDK's `on_select` callback confirms these recording metadata values when
Preview opens the source. Unsupported robot/hand/action identities are rejected;
this callback does not initialize, move or reconfigure the physical robot.

**HOLD and C keep the same Loop recording running.** Joint state, latest
command values, camera frames and status continue. A held or stale command is
not relabeled as a fresh demonstration. No observation is copied into action
to fill a missing message. A paused arm can drift or a keyboard operator can
move the base/head/lift; continuous observations preserve those events.

| Group | observation | action |
|---|---|---|
| left_arm / right_arm | measured 7 joints each | final gated 7-joint trajectory |
| left_hand / right_hand | measured 20 joints each | native ROBOTIS retargeted 20-joint trajectory |
| head | measured 2 joints | gated keyboard trajectory |
| lift | measured 1 joint | gated keyboard trajectory |
| mobile | measured odometry vx, vy, wz | gated cmd_vel vx, vy, wz |

### HX5-D20 fingertip tactile

The read-only adapter subscribes to `/left_hand/finger_pressures` and
`/right_hand/finger_pressures` (`robotis_interfaces/msg/HandPressures`). It
uses the existing follower pressure broadcasters and never opens a second
serial connection or starts a tactile grasp controller. The official current
SH5 follower launch includes `pressure_l_broadcaster` and
`pressure_r_broadcaster`; older installed workspaces may not have these.

Each hand's existing observation/action joint-position vectors still contain
20 joints. The same source adds these channels beneath
`ffw_sh5_vr.observation.left_hand` and `.right_hand`:

| Suffix | Entries |
|---|---|
| tactile_pressure | 45 unsigned raw readings (0..255); 5 sensors x 3 x 3 |
| tactile_status | received, valid_message, fresh, observed_receipt_hz |
| tactile_timing | receipt age in seconds (-1=missing), ROS header stamp in seconds (-1=missing), local receipt sequence |

Pressure ordering is `finger_l_sensor1..5` / `finger_r_sensor1..5`, each with
`Present Pressure 1..9` in row-major order (top to bottom, left to right).
The adapter resolves both sensor names and pressure names; array arrival order
does not change this layout. Sensor numbers are kept verbatim rather than
guessing their anatomical finger names. Missing/duplicate sensors, invalid
interface names and values outside 0..255 invalidate that hand's sample.
Malformed or older-than-0.5-second pressures are `None`, not fabricated zeros.
Status/timing continue even when pressure is missing; `valid_message` describes
the last message, while `fresh` additionally checks receipt age. Receipt Hz
measures callback arrival, not the hardware sampling clock or sensor health.

The combined source sends the latest raw readings on its existing 30 Hz
recording clock, alongside joint values, including during C/HOLD. It does not
preserve every upstream frame if the pressure publisher runs faster; header
stamps, receipt sequence and arrival rate make reuse/downsampling visible.
Raw pressure is not calibrated Newton force. No baseline subtraction or
RH5DG2 force/proximity/palm conversion is applied. These are robot channels in
the existing Preview/recording source; this change does not add a custom hand
heatmap widget to the Loop application.

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

`ffw_sh5_vr.teleop.*` contains:

| Channel | Vector entries |
|---|---|
| flags | active, paused, manual_active, fault, status_fresh |
| phase | WAIT_CALIBRATION=0, ALIGN_REQUIRED=1, SETTLING=2, WAIT_START=3, RESUME_SEED=4, ACTIVE=5, HOLD=6, FAULT=7; unknown=-1 |
| action_fresh | left_arm, right_arm, left_hand, right_hand, head, lift, mobile |
| action_valid | same group order; arms/hands require ACTIVE, auxiliaries allow manual_active |
| ages | joint receipt, gate receipt, PC intent receipt in seconds; missing=-1 |
| clock | age of PC clock anchor, robot-minus-PC wall-clock difference in seconds |

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

## Keyboard in the Loop window

| Key | Behavior |
|---|---|
| W / S | base forward / backward |
| A / D | base left / right translation |
| O / P | yaw left / right |
| arrows up / down | head tilt up / down |
| arrows left / right | head pan left / right |
| U / I | lift up / down |
| C | start / HOLD / resume arms and hands; clear current motion keys; recording continues |
| V | on Loop review screen: tap = Save Failed; hold = discard, preserving Loop's original C behavior |
| Shift+A or Loop's button | start recording; plain A belongs to base translation in VR mode |

This mode uses X11 passive grabs only on the Loop window. Motion is disabled
outside that window and on focus loss, modifiers, input expiry or missing
measured state. Release keys before refocusing. Operate in the Recorder view;
disable the operator session before typing configuration text with these keys.
While this bridge runs, plain C is consumed before Loop receives it. V forwards
Loop's original C press/release, including its hold duration. Loop's compiled UI
still labels these review actions C / Hold C; use V / Hold V in this VR mode.
The on-screen buttons retain their normal behavior. No recording API is invoked.
C starts or resumes only with fresh bilateral hand/body tracking and a valid
robot profile. It reanchors to measured wrists and uses the existing startup
checks; rejected starts are logged, never queued for later automatic activation.
The bridge owns no follower publishers; all motion passes through the existing
hardware gate, measured-state checks, physical limits and a 0.35s manual lease.
Keyboard speeds are 0.15m/s translation, 0.3rad/s yaw and 0.25rad/s head,
additionally bounded by the physical URDF and measured lead limits. U/I follows
the installed physical leader's full-stick lift jog: measured position +10mm /
-10mm at 100Hz, with physical position and gate slew limits. This is a target
offset, not a fixed lift speed; the former separate 0.04m/s integration is gone.

Keyboard head motion selects an optional 57-scalar-joint collision model:
14 arms + lift + 40 measured fingers + 2 measured head joints. The native arm
QP has zero velocity bounds for the fingers/head and consumes their measured
positions; it does not generate their commands. Existing 55-joint hand profiles
remain supported. Keyboard head/lift target checks use cached collision FK.
Possible box contacts caused by these auxiliary targets are refined with the
hashed original collision meshes. This avoids a known head/body box overlap at
the measured ready pose. The arm QP keeps its conservative box geometry.

Base movement first holds arms, because wheel poses are frozen in the arm
collision profile. Releasing keys keeps the gate/controller running: keyboard
base/head/lift controls remain available, with arms held. After repositioning,
press C to request a fresh profile. The PC first stops manual commands and waits
for a fresh disarm acknowledgement; only then does the robot restart its own
controller to collect the profile. Motion keys are blocked during this explicit
recapture. Loop continues recording. Wait for `[BASE RECAPTURED]`, release motion
keys and press C again to re-anchor and resume arms. Rejected recaptures now
report the actual blocking status instead of one generic message. Stale gate or
missing-service waits time out; they do not queue a later automatic arm start.
C is a software hold, not physical torque release or a hardware emergency stop.

## Arm cadence and the SG2 / hand paths

The old SG2 controller path has 15 controller coordinates. SH5 hands add 40
measured finger coordinates (55 total); keyboard head control adds another 2.
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
