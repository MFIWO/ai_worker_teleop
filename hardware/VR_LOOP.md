# SH5 VR teleoperation and continuous Loop recording

The two commands below run in separate PC terminals. The PC address must match
the Vuer certificate. Open the installed Loop application first for `--loop vr`.
The PC command waits up to 120 seconds for the robot's Zenoh port 7447 before
starting either ROS process. Enter the robot command in the other terminal
while it displays `[WAIT ZENOH]`; either command may be entered first. A timeout
means the robot session/router still needs attention. Ctrl+C cancels the wait.

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash teleop.sh pc --pc-ip 192.168.6.2 --robot-ip 192.168.6.50 --loop vr --input controllers
```

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash teleop.sh robot --pc-ip 192.168.6.2 --robot-ip 192.168.6.50 --loop vr --input controllers --bringup
```

`--bringup` starts a missing Zenoh router and SH5 follower with
`init_position:=false`. The native hand/head/lift/steering initialization
trajectories are skipped. Once stable feedback is available, only the two
seven-joint arms follow the existing xr_tele quintic ready trajectory over five
seconds, to `[0,0,0,-1.57,0,0,0]` per arm. No hand position trajectory is sent
by this ready mover. Native driver initialization and the follower's existing
hand current commands still run; this option does not turn hand torque off.
This is a startup action, not part of HOLD/resume. An existing follower is
reused without motion; add `--ready-pose` to the **robot** command to request
the same arms-only move explicitly. Stop old initialization executors and VR
controllers first. Motion failure prevents the VR controller from starting.
Without `--bringup`, router and follower must already run.

The foreground session owns the VR controller and optional Loop streamer. On
Ctrl+C or terminal closure first stops command production. After a minimum
three-second exit grace, the SH5 follower stops and a dedicated helper disables
TableSync and sends Torque Enable(64)=0 to each of the 40 finger motors using
direct motor access. It reads back each torque register; only 40 verified zeros
produce `[HAND TORQUE OFF] Verified 40/40 finger motors`. The delay starts on
robot-session exit detection; network/PC-heartbeat detection, launch cleanup
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
also ends/expires it on the container side. The robot additionally detects a
previously connected PC Vuer heartbeat disappearing (1.5s). B+Y/V HOLD and Quest
tracking loss retain intent heartbeats and do **not** trigger torque OFF.
Terminal disconnect logs are saved to the session's `exit.log` (PC side:
`/tmp/sh5_pc_session_exit.log`). Allow cleanup to finish before restarting.

The helper verifies that no other process owns the serial device. It refuses
a second bus connection, unknown SH5 motor mappings and non-Direct-Control hand
mode. It never clears errors, enables torque or changes operating mode. Failure
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

### Hand mode 2: A/B pinches, left-grip arm clutch, right-grip ready recovery

Pass `--input controllers --hand-mode 2` on **both** PC and robot commands.
This changes ONLY the explicit SH5 controller hand mode 2; controller hand mode 1
and optical input retain their original finger behavior.

| Controller input | Mode-2 behavior |
|---|---|
| X tap (left A button) | Trigger = original full grip |
| Y tap (left B button) | Trigger = original C-shape angled entry/grip |
| A tap (right A button) | Trigger = **thumb + index + middle** tripod pinch; thumb joints 3/4 bend farther than former Y pose |
| B tap (right B button) | Trigger = **thumb + ring + little** ulnar pinch; index/middle remain open |
| Left grip held | **Clutch arms ON**: both wrists follow controllers while grip is held |
| Left grip released | **Clutch arms HOLD**: controller hand displacement no longer drives wrists, but measured lift displacement still carries arms up/down |
| Right grip pressed | Request a **both-arm initial ready pose**; stops arm teleop and invokes the robot-owned recovery action, then a fresh Cyclo restart |
| A+X after ready recovery | Release both grips and A/X first, then press **A+X** to capture both real wrists and current VR zero; hold left grip to resume arm teleop |
| Y+B | Existing full HOLD command (both buttons); does not select B pinch |
| Both stick clicks | Toggle the existing HEAD and MOBILE joystick modes |

Mode-2 grips **never command pinch**; only selected trigger profiles close
fingers. Each side captures the current preset when its trigger starts closing,
retains it while held, and releases it at zero trigger. A solo face-button tap
commits only after 150 ms of quiet release; A+X and B+Y start/stop chords and
partial releases never select a hand pose. Mode 1 still uses proportional grip
for thumb/index/middle pinch.

A and B pinch poses start with a measured **thumb preparation stage**, then
close the selected fingers through the existing 85%-of-URDF velocity cap and
60 ms controller target smoothing. All new thumb/pinch joint angles are
**simulation candidates**, not verified human-robot pinch contacts. Validate
fingertip spacing, self-collision and grasp pressure in PhysX and tune
`hardware/hx5_controller_poses.json` before a physical pinch test.

**Right-grip ready recovery requires the new robot launcher.** The PC disarms
first and waits for an explicit fresh gate acknowledgement. The robot launcher
then stops the old Cyclo/gate process group **before** executing the existing
five-second arms-only `hardware.ready_pose` action and starts a new Cyclo/gate
process group afterward. It reports `READY` only after the new gate provides
fresh healthy disarmed feedback. A+X remains blocked during recovery, and
failure stays disarmed; inspect session logs instead of repeatedly forcing
motion. The original SH5 follower remains running; the recovery does not move
lift, head, mobile base, or fingers.

The lift-follow correction is shared by optical and controller input. It uses
**measured** `lift_joint` feedback to move the two wrist targets, their
elbow/workspace hints and calibration anchors by the same vertical displacement;
it does not apply the command target twice or change VR orientation zero.
In mode 2, releasing the left grip freezes XR relative motion, **not** the
lift/body frame, so lowering lift carries the hands downward.

SH5 right-stick functions are exclusive: HEAD mode controls lift with vertical
stick travel; MOBILE mode controls base yaw with horizontal travel and ignores
lift input. Both stick clicks toggle modes. The existing left-stick head/base
mapping and activation checks remain in effect.

The final 40 finger targets pass through the robot gate at 100Hz, capped at
85% of each physical URDF velocity limit. Mode 1 and optical hand input retain
the same cap. This reduces speed modestly; it does not reduce holding torque
or prove thermal protection. HOLD retains measured positions rather than
disabling torque. Stale controller input stops updates.

For the prior optical hand/Loop keyboard workflow, use `--input hands` on both
commands. That mode retains V start/HOLD/resume and the C review-key mapping.

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

**HOLD and V keep the same Loop recording running.** Joint state, latest
command values, camera frames and status continue. A held or stale command is
not relabeled as a fresh demonstration. No observation is copied into action
to fill a missing message. A paused arm can drift or a keyboard operator can
move the base/head/lift; continuous observations preserve those events.

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
recording clock, alongside joint values, including during V/HOLD. It does not
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

## Read-only hardware finding (2026-10-23)

With the follower stopped and no owner of `/dev/follower`, board-only READs
returned Hardware Error Status(71)=64 (0x40) and Bus Watchdog(8)=255
(signed -1) on right ID110 and left ID140. Input voltages were 28.62V and
28.48V. The [official hand control table](https://ai.robotis.com/hands/control_table_hands.html)
identifies these as Bus Watchdog errors after an interruption of TableSync
communication. They may have occurred when the follower stopped; this reading
does not establish the cause of earlier in-motion flashing or finger heating.
That diagnostic used READs only; no error clear, torque change or physical
motion was performed during the diagnostic. The subsequent exit helper is
explicitly designed to WRITE only TableSync OFF and finger Torque OFF.
Individual finger motor temperatures and overload flags were not obtained.

## Robot eth0 boot conflict

On 2026-10-23, this robot's `/etc/netplan/interfaces.yaml` used `networkd`
and assigned eth0 `192.168.6.2/24`, while its saved NetworkManager profile
assigned `192.168.6.50/24`. Udev reported `NM_UNMANAGED=1`; the boot log
left eth0 unmanaged until manual activation. The PC log also recorded a
conflict for its `.2` address from the robot's MAC. PC USB Ethernet already
had autoconnect enabled.

The robot HOST helper is deliberately separate from motor/source installation:

```bash
ssh -t robotis@192.168.6.50 \
  'sudo python3 ~/ai_worker_teleop_sh5/hardware/fix_robot_ethernet.py'
```

It checks the narrow known Netplan layout and active static-IP NM profile,
backs up the configuration, replaces the obsolete networkd definition with
[Netplan's NetworkManager renderer](https://netplan.readthedocs.io/en/stable/nm-all/),
and preserves the existing `.50` connection. Autoconnect stays enabled with
[unlimited retries](https://www.networkmanager.dev/docs/api/latest/settings-connection.html).
It runs `netplan generate` for the next boot and does not restart networking,
apply a live address change, reboot, or start any robot controller. A backup
and restore script path are printed; errors restore the previous settings.
The final root application requires the robot's sudo authentication. Boot-time
reconnection still needs confirmation after an operator-controlled reboot.

Applied to `ffw-SNPR48A1114` on 2026-10-23. The retained eth0 profile reads
`192.168.6.50/24`, autoconnect=yes, retries=0. Backup:
`/var/backups/ai-worker-ethernet/1777511249356407928`. SSH remained connected;
a real reboot/boot-time reconnection has not been exercised.
