# SG2 / SH5 arms-only hardware teleoperation

Added 2026-10-14 on the existing `robotis-applications` branch. This is an
**opt-in hardware setup**, not a claim of a completed physical robot test.
The original SG2/SH5 publishers and all simulation modes remain available.

## [CHANGED 2026-10-14] Stationary SG2 startup / joint6 correction

The recorded failure at ROS time 1791951929.472 was a hardware-gate joint6
tracking fault, not a reference-checker fault or misaligned Quest hands. It
occurred 0.413 seconds after native Cyclo began emitting commands.

Three corrections are now separated:

* SG2 selects a separately built **numeric-only** Cyclo core. OSQP absolute and
  relative convergence tolerances are both 1e-5, applied on every solver init.
  The objective, 1000 slack cost, CBF, collision pairs, velocity/position bounds
  and Solved-status requirement are unchanged. The original installed sources
  and binary remain unchanged. No simulation joint expansion is loaded.
* The gate preserves the original single-point, zero-header, t=0 stream instead
  of turning every 20ms update into a new 100ms interpolation. Multi-point or
  nonzero-duration inputs are rejected instead of silently compressed.
* PC logs identify hardware-gate faults separately from true reference faults.
  The first failing joint target, actual angle, prior output and input age are
  retained in gate_status/fault_detail and the robot log before hold clears the
  pending commands. Normal `driver_managed` mode uses URDF position bounds
  and velocity slew limiting without additional following-error shutdowns.
  The 50mm/20deg PC guard and inherited 250mm/90deg guard are disabled in the
  hardware publisher. Strict commissioning retains its URDF-speed-derived
  tracking budget. Feedback/lease expiry, motor errors, joint bounds, malformed
  tracking, collision/CBF and reference checker still stop the session.

Hardware resume follows xr_tele's U-release anchor reset: Y+B holds, release
X/A then press X+A to anchor current hands to fresh measured wrists. Returning
to the old hand pose is unnecessary. After the controller's actual activation
delay and ramp complete, current hand poses are zeroed again to the unchanged
held robot targets. Hand motion during preparation is discarded. In WAIT_START,
a fresh X+A also rezeros current hands instead of requiring the previous hand
position. Both wrists commit together; heading, elbow/workspace references and
held robot goals stay unchanged by this final rebase. Look for
`[XR RESUME INPUT_ZERO]` followed by `[XR RESUME READY]` before moving.
Actual startup timing is queried from Cyclo and logged as
`[HARDWARE STARTUP ACTUAL]`. Distant old goals are interpolated to the
measured seed while controller and gate remain inactive, preserving the
reference checker's jump bounds. Startup then uses 1s activation plus 1s ramp;
30mm/15deg is the stillness/alignment window during this stationary startup.
On a fault, a fresh X+A requests a manual gate reset. It requires a disarmed
fresh PC lease, verified model, fresh joints, healthy motor telemetry and valid
measured joint bounds. An ordered post-reset gate acknowledgement unlocks the
PC and starts measured-anchor preparation. Persistent errors refuse reset or
stop again; Y+B cancels a pending restart. The service is
`/xr_hardware/reset_fault` (`std_srvs/srv/Trigger`). SIM modes retain fixed mapping.

Hardware freshness uses **local monotonic receipt time**. There is no comparison
of a PC wall clock with a robot header or vice versa, including initial motor
prechecks and snapshot capture. Sender-local increasing header stamps reject
duplicate/out-of-order JointState, DXL and TF samples; PC leases and gate status
use session IDs plus sequence numbers. TF freshness is recorded by /tf receipt,
not by reading cached transforms. Active robot feedback and Cyclo output gaps
wait up to 5s from last receipt. After 0.5s the gate holds its last output and
zeros base velocity; Cyclo pauses QP computation until measured feedback returns.
The session resumes automatically within 5s, without accumulating integration time.
Arming and fault reset still require fresh feedback. Active PC intent gaps also
wait up to 5s, holding outputs and zeroing base velocity after 0.5s.
Cyclo receipt ages, activation delay and ramp use `RCL_STEADY_TIME`; message
stamps keep ROS time. Existing joint tracking, CBF and URDF policies remain.
Remote packet
generation age is intentionally not inferred from timestamps in this mode.

At a joint boundary, solver overshoot up to 1e-5rad is snapped to the exact
URDF lower/upper value before forwarding. This matches the QP numerical
tolerance and prevents a rounded `-0.000000` at joint2=0 from latching a
session fault. Commands remain inside URDF bounds; larger violations still
latch and report the target/violation with full floating-point precision.

The existing PC/robot commands also restore SG2 auxiliary controls in ACTIVE:
left stick=head, right stick=lift; both stick clicks toggle left=base XY and
right=yaw+lift. Each trigger uses the original SG2 VR command scale 0..1.3rad.
The robot profile labels this `original_sg2_vr_1.3rad` and retains the URDF
upper value (1.1rad) as `urdf_upper`; head/lift ranges and auxiliary speeds
continue to come from the physical URDF.
Commands travel in the PC lease to the robot gate. Grippers join the gate's
single arm output; head/lift use the existing follower relay topics. Y+B,
tracking loss, gate faults and PC lease expiry stop the base and clear auxiliary
targets. Measured lift translation moves the saved wrist/workspace vertically
without resetting the input anchors or head basis. No legacy Vuer is required.

Build the SG2 numerical library once **before** operating the follower, inside
the robot ai_worker Docker with the existing ROS/workspace environment sourced:

```bash
python3 -B /workspace/ai_worker_xr/hardware/build_qp_precision.py
```

`run_controller.sh sg2` verifies its source/library hashes and actual ldd path.
The current robot installation has already been built. SH5's selected runtime
library is unchanged by this SG2-specific launcher change.

The original ready angles, driver_managed torque policy and X+A/Y+B flow are
unchanged. The original ready configuration is inside the CBF margin; its
unmodified CBF therefore still commands a bounded posture adjustment even with
held wrist goals. The corrected solve substantially reduces the unrelated
numerical drift; it does not disable collision avoidance to hold that posture.

Validation on the installed SG2 model: an 8s held-goal numerical trace reduced
peak wrist drift from 41.47mm to 12.88mm and final error from 5.52mm to 0.093mm.
There was one unsolved cycle, held without publication exactly as in the native
controller; maximum consecutive gap was one cycle. Mean/max isolated QP solve
time was 9.21/49.19ms in that run, **not a hard 100Hz real-time guarantee**.
Real Cyclo + real rclpy gate on localhost/domain232, with a synthetic first-order
follower (tau=0.15s), passed stationary startup (10.14mm maximum wrist drift) and
a 20mm forward stroke (19.998mm both arms). This is not a physical motor, Quest,
or dynamics validation. Real hardware tracking and timing still need an operator
trial. No physical follower or arm motion was started while making this change.

## [CHANGED 2026-10-14] Normal path and optional strict commissioning

`run_xr_hardware_pc.sh` and `hardware/run_controller.sh` now select
`readiness_policy=driver_managed` explicitly. This keeps the existing native
follower responsible for torque management. It does not enable torque, repair
the driver, fabricate a positive torque bit, or independently verify torque ON.

The ready-pose precheck, passive snapshot and command gate still require valid
fresh DXL read data, expected arm IDs and no communication/hardware errors.
Reported torque bits are diagnostic in this normal path, not an extra mandatory
authorization protocol. The status includes `reported_torque_all_on` separately
from `motor_read_healthy`, and `physical_torque_verified` remains false.

The additional *startup* requirement for the full native CBF margin is also
separated from the controller's actual constraint. A positive initial modeled
clearance below that margin is reported in normal mode; modeled arm-relative
contact/penetration and invalid/nonfinite geometry still reject startup. The
original Cyclo CBF distance, collision pairs, joint bounds and reference checker
are unchanged. No recovery trajectory is synthesized and no initial posture is
silently altered. The runtime startup/tracking checks may still stop a motion;
passing this preflight is not a physical safety or tracking certificate.

Both sides must select the same policy. Old profiles retain `strict_torque`;
generate a fresh profile using the updated robot launcher after updating files.
An old robot profile does not silently become a normal-mode profile.

The additional old checks remain available by passing `--strict-commissioning`
to BOTH launchers. The named `run_xr_commissioning_pc.sh` and
`hardware/run_commissioning_controller.sh` wrappers select this mode explicitly.
This mode still requires reported all-ON torque and the full initial CBF margin.
It is not required for the normal workflow.

### [CHANGED 2026-10-14] Three terminals: follower, ready + Vuer, controller

Keep the already-established native follower running in the first terminal.
Stop old Vuer/Cyclo processes before preparing again; do not stop the follower.
The new `--start` shorthand is exactly `--ready-pose --arm`, not a new mover:

```bash
# Second terminal, PC applications Docker: existing ready move, then Vuer here.
# .101 is the PC HTTPS server; .102 is the ROBOT Zenoh router.
bash /root/ros2_ws/src/robotis_applications/run_xr_hardware_pc.sh \
  sg2 --start --server-ip 192.168.5.101 --robot-ip 192.168.5.102

# Third terminal, ROBOT ai_worker Docker, AFTER [READY POSE COMPLETE]:
bash /workspace/ai_worker_xr/hardware/run_controller.sh sg2
```

With `--server-ip`, local certificate IP/date/key checks happen BEFORE movement.
A failed or interrupted precheck/ready move does not start Vuer. The successful
move prints the next robot command, then starts Vuer in the same terminal.
Before the robot controller starts, Vuer's feedback-not-ready message is expected;
the operator still uses X+A only after the controller is ready. No automatic
controller activation, SSH launch, process termination or hardware-guard change
is added. `--arm` alone still means Vuer only; `--ready-only` still exits after
the ready move. Logging/Zenoh terminals, if used separately, remain separate.

### [ADDED 2026-10-14] Vuer certificate for the PC, not the robot

Generate on the PC applications Docker using the existing installed helper.
This changes server TLS files, not robot settings. Stop the old Vuer first;
it keeps its loaded certificate until restarted. From the PC HOST terminal:

```bash
docker exec -it robotis-applications bash -lc '
  set -e
  CERT_DIR=/root/ros2_ws/src/robotis_applications/robotis_vuer/robotis_vuer
  BACKUP="/root/vuer_cert_backup_$(date +%Y%m%d_%H%M%S_%N)"
  mkdir -m 700 "$BACKUP"
  for f in cert.pem key.pem; do
    if [ -f "$CERT_DIR/$f" ]; then cp -p "$CERT_DIR/$f" "$BACKUP/$f"; fi
  done
  cd /root
  bash /root/gen_cert.sh 192.168.5.101
  chmod 600 "$CERT_DIR/key.pem"
  openssl x509 -in "$CERT_DIR/cert.pem" -noout -checkip 192.168.5.101 -dates
'
```

The CA is installed inside this container by `mkcert -install`, not automatically
on the PC host or headset. Quest's local-development certificate warning is a
separate client trust decision. Open ONLY this known local server:

```text
https://192.168.5.101:8012?ws=wss://192.168.5.101:8012
```

ROBOTIS documents Advanced -> proceed for the local-development warning. Refresh
and re-enter VR after restarting Vuer. Never share `key.pem` or `rootCA-key.pem`.
For CA trust installation elsewhere, export only the public `rootCA.pem`.
References: ROBOTIS AI Worker VR teleoperation guide and FiloSottile/mkcert README.

The original `READY_Q=[0,0,0,-1.57,0,0,0]`, ready-move action acceptance,
measured convergence, manual X+A/Y+B, fixed calibration, speed limits, lease
watchdog and tracking-error checks remain. No physical movement was executed
while making this change. SH5 still uses its existing preparation workflow;
`--ready-only` is SG2-only and no SH5 hand/finger control is added.

## Earlier local robot audit, 2026-10-14 — historical strict-mode findings

The following audit describes the previous strict commissioning path. Its
mandatory-torque and startup-margin conclusions are not prerequisites imposed
by the normal path above. The underlying driver findings are not claimed fixed.

The actual `ffw-SNPR48A1112` installation was inspected over SSH. The hardware
container is `ai_worker` (`robotis/ai-worker:2.2.8`); the other container named
`robotis-applications` on that host is NOT the hardware workspace.

The earlier setup synchronised clocks through a PC-only NTP endpoint. This is
optional with the current receipt-time policy. Historical setup:
from the PC HOST after restarting its container:

```bash
bash hardware/start_time_server_pc.sh 192.168.5.102
```

The server uses chronyd `-x`, leaves the host clock owner unchanged, and admits
only the robot IPv4 address. A recreated PC image needs the dependencies now
listed in `docker/Dockerfile`. The installed container was updated without
recreating it. Robot timesyncd uses `192.168.5.101`; this depends on those LAN
addresses remaining valid.

Two independent blockers remain. First, the installed driver's published
`dxl_torque_state_` map is never populated. Its reported False is not reliable
proof that physical torque is OFF. Second, the old ready target with both
elbows -1.57 rad and all shoulders zero has only 12.1097 mm native-model
clearance, below the unchanged 20 mm requirement.

The prior buffered-service TorqueEvidence fallback has been DISABLED: an
actual-source offline reproduction showed old queued ON replies authorising
readiness while all simulated physical registers were OFF. It also adds
blocking service work to the driver's own hardware read loop. The current
compatibility object performs no polling and never grants evidence. This is
a safety correction, NOT a repair of the original driver's feedback.

A candidate with left joint2 +0.04 rad and right joint2 -0.04 rad passed the
installed model's endpoint check at 25.2688 mm. It was not deployed or executed.
The sampled path from saved home starts at 7.5 mm clearance and is not within
20 mm for its entire length; no continuous/physical safety claim is made.

The proposed private driver-feedback overlay was not applied. No physical
follower, ready-pose action, torque-enable service or VR activation was started
during this audit. Do not interpret the commands elsewhere in this README as
an assertion that this robot is now motion-ready.

Detailed audit and original-file backups are stored on the PC under
`~/Downloads/sg2_commissioning_20261014/`.

## What carries over from simulation

The same measured-start spatial wrist delta, fixed horizontal heading,
immutable calibration, anchor-ray workspace limiting and passthrough alignment
markers are reused. There is no arbitrary 0.5m-from-anchor shutdown. A stroke
through the captured wrist remains on that ray when its length is limited.
This does not enforce a straight arbitrary trajectory, compensate high-speed
tracking lag or make all seven joints return to their original posture.

Hardware differences are deliberate:

| Setting | Hardware commissioning |
|---|---|
| ROS domain | 30; one selected physical robot at a time |
| Wrist goal filter speed / angular speed | 0.30m/s / 1.50rad/s |
| Arm joint velocity bound | per-joint URDF velocity (inspected SG2:4.8rad/s) |
| Joint position bounds | SG2 physical URDF; SH5 reviewed reduced model |
| SG2 elbow joint4 upper | physical URDF 1.0786rad |
| Measured geometric workspace | original0.94 reach fraction, not simulation0.995 |
| Wrist position/orientation weights | existing10/1 |
| Elbow auxiliary cost | 0; wrist-centric7-DoF solve |
| QP / collision / CBF / reference checker | original installed implementation retained |
| SG2 gripper / head / base / lift commands | original controls restored via gate; ACTIVE only |

Consequently the hardware setup is **not expected to straighten as far as the
expanded simulation experiment**. Changing the real elbow limit requires a
separate mechanical and manufacturer-limit review; it is not done here.

## Robot-side boundary

```text
Quest controllers -> HardwareXRPublisher (PC, PoseStamped only)
    -> original Cyclo VRController + original reference_checker (robot)
    -> private /xr_hardware/{sg2|sh5}/{left|right}/joint_trajectory
    -> HardwareCommandGate (robot)
    -> existing physical arm JointTrajectoryControllers
```

The gate verifies the selected live `/robot_description`, its snapshot hash,
arm geometry, unique topic writers, measured joint state, native joint limits,
output rate and a fresh PC intent lease. It forwards **seven arm joints only**;
`hand:=false` by itself does not prevent old Cyclo versions appending a gripper.

Real Dynamixel-read health is required from `/ffw_follower/dxl_state` with
`dynamixel_interfaces/msg/DynamixelState`. Arm motor IDs are right1..7 and
left31..37, verified against GPIO definitions in the expanded description.
Explicit bus/ID/hardware errors or a PC lease older than5s latch the gate.
Missing/replayed feedback does not refresh receipt age: gaps hold targets after
0.5s and latch only after 5s. Reported torque is additionally
required only in strict commissioning. Fresh joint-state
timestamps alone are not enough: the hardware driver can temporarily publish
cached joint values during a motor-read failure.

Cyclo receives a canonical-order joint-state stream from the gate because its
installed callback caches name indexes from the first message. The gate never
renames a missing joint or invents a measured position.

After a gate fault, resolve the cause, release X/A and press X+A for a checked
reset/retry. Normal Y+B stop/X+A resume captures measured anchors.

**This software gate is not a certified emergency stop.** A process hard-kill,
host power loss or motor-driver failure is not covered by a Python stop callback.
Use the physical emergency stop and the normal hardware driver's watchdogs.
ROS node names/domain IDs are not authentication: keep the robot network trusted
and isolated from unrelated command publishers. Do not run two robots in the
same un-namespaced domain30 graph. Current freshness checks do not require
PC/robot wall-clock synchronization.

## SH5 model handling

The installed Cyclo package only supplied a reduced SG2 control model. The
actual SH5 description has continuous wheel joints, head joints and40finger
joints, so it cannot simply be passed to the scalar15-DoF Cyclo solver.

`sh5_model.py` builds a **separate** model from the live expanded SH5 description:
all links, hand geometry and inertials are initially retained;14arm joints plus
the lift remain scalar DOFs; all other movable joints are frozen at their
measured snapshot transforms. The lift controller velocity bound stays0.
Arm bounds are the conservative intersection with the existing Cyclo model.
Compatible original SRDF pairs, original adjacencies and fixed-body-cluster
exclusions are recorded. Omitted original exclusions are listed, not guessed.

The exact65SH5 collision meshes took about2.71s per distance evaluation on the
checked PC, unsuitable for a100Hz controller. `collision_primitives.py` therefore
encloses **each** supplied collision mesh in a conservative box, with original
collision rotation and correctly transformed center. Every mesh vertex is
checked for containment, scale is applied exactly once, and a per-mesh manifest
is retained. Visual meshes, link geometry and collision-pair selection remain.
This is an enclosing approximation, **not exact mesh collision geometry**;
it may reject otherwise feasible poses. Initial clearance is checked again.
The isolated benchmark fell to about0.38ms mean per distance evaluation over20
calls; this is not a100Hz end-to-end timing guarantee on the robot computer.

The gate continuously checks frozen joints against the snapshot (0.01rad/m
numerical tolerance). Do not operate the SH5 hand, head, wheels or another
controller while this arm-only model is active. To change their posture, stop,
use the established hardware workflow, then generate a new snapshot/profile.
This is **not** optical finger retargeting and does not start hand controllers.

## Install on the robot host — once per update

Use this same branch on the robot computer, not `main`. For a new checkout:

```bash
git clone --single-branch --branch robotis-applications \
  https://github.com/MFIWO/ai_worker_teleop.git "$HOME/ai_worker_xr"
cd "$HOME/ai_worker_xr"
bash hardware/install_robot.sh
```

For a previously clean checkout, use `git pull --ff-only` then rerun the install
script. The script copies named source files into
`ai_worker:/workspace/ai_worker_xr`; it does **not** start a robot, issue a ready
pose, build the simulation precision overlay, or copy credentials/logs/models.
Set `AI_WORKER_CONTAINER` only if the established robot container has another name.

The robot image needs the same installed Cyclo/ffw_description packages,
`dynamixel_interfaces`, Python `numpy`, `scipy`, `yaml`, and `pinocchio`.
The checked image is `robotis/ai-worker:2.2.7`. An unknown model/library is an
error to review, not permission to bypass validation.

## Robot terminal — choose SG2 or SH5

Keep the **already-working physical follower** running using its established
procedure. Stop the previous Cyclo VR controller and Vuer before switching.
This wrapper intentionally does not start `ffw_*_follower_ai.launch.py`: that
launch can initialize arms, wheels and SH5 finger effort even with no VR hand node.

Inside the robot's AI Worker Docker:

```bash
export ROS_DOMAIN_ID=30
bash /workspace/ai_worker_xr/hardware/run_controller.sh sg2
```

For SH5 use the same command with `sh5`. It first passively obtains the actual
description, increasing fresh motor/joint data and a stable measured posture.
It generates a new profile, verifies collision mesh resources and evaluates
clearance for collision pairs affected by arm movement before starting Cyclo.
Normal mode reports a positive clearance below the native CBF margin; strict
commissioning requires the full margin before launching.
The gate and controller start **inactive** until a checked manual X+A.

For a **passive-only** check, add `--check-only`. No Cyclo or gate is launched.
If the measured initial pose is outside original limits or has modeled
arm-relative contact/penetration, it refuses rather than moving automatically.
Use the robot's known-good ready-pose workflow, keep clear, then retry. The
controller's collision threshold is not modified by either launcher.

Generated robot-local files are stored under
`/workspace/ai_worker_xr_profiles/{model}_<unique timestamp>/` and are not committed.

## PC / ROBOTIS Applications terminal

The PC Docker must mount this branch at
`/root/ros2_ws/src/robotis_applications` (as in the existing setup).

Observe-only startup, X+A cannot enable motion:

```bash
export ROS_DOMAIN_ID=30
bash /root/ros2_ws/src/robotis_applications/run_xr_hardware_pc.sh sg2
```

After checking the correct robot, unobstructed workspace and emergency stop,
restart the PC publisher in explicitly armed commissioning mode:

```bash
export ROS_DOMAIN_ID=30
bash /root/ros2_ws/src/robotis_applications/run_xr_hardware_pc.sh sg2 --arm
```

For SH5 replace `sg2` with `sh5` on **both** robot and PC commands. Do not also
run the old `model:=sh5` optical-hand publisher. The new mode uses Quest motion
controllers for arm wrists only. The existing HTTPS/WSS8012 address and local
certificate remain; certificates and private keys are not in Git.

Refresh the Quest page. First X+A captures calibration; wait for
`[XR RESUME READY]` / `ACTIVE`. If `WAIT_START` appears, match the white tracked
points **and** axes to the coloured targets, release X/A then press again.
Y+B stops. Repeated X+A does not re-zero an active session. Begin with slow,
small arm motions; larger goal errors still stop commissioning.

## Passive log in another robot Docker terminal

```bash
bash /workspace/ai_worker_xr/hardware/log_arms.sh sg2
```

Use `sh5` for SH5. It uses the exact generated URDF selected by the controller
and writes fresh `arms.csv`, `events.jsonl`, `metadata.json` under
`/workspace/ai_worker_xr_logs/`. Real TF is forward kinematics from measured
joints, **not independent optical measurement of the physical end effector**.

## Validation scope

The implementation has pure geometry/contract tests, real-rclpy watchdog and
publisher tests, and actual Cyclo model-load checks for SG2 and SH5 on isolated
domain232. SH5 reduced-model FK tests include nonzero frozen finger poses and
verify every link transform against an independent FK implementation.

SG2 domain232 integration uses actual Cyclo/relay/PhysX but **synthetic** controller
inputs and **synthetic** DXL health. Such a run is not a physical motor test.
Validation profiles are marked and rejected by the hardware path. Test-only
driver fixtures are not copied by the installer.

Latest completed integration checks:

Release regression:100XR/publisher tests,4existing configuration tests and45
hardware contract/gate/model/primitive tests passed (149total). These include
all65SH5 mesh-to-box containment checks and independent reduced-model FK cases.

| Model | Feedback backend | Result |
|---|---|---|
| SG2 | Actual PhysX + Cyclo + hardware gate | 7checks passed;40mm input,39.94mm measured forward movement; PC lease-loss stop verified |
| SH5 | Pinocchio kinematic follower fixture + actual Cyclo + hardware gate | 7checks passed;40mm input,39.73mm FK displacement; PC lease-loss stop verified |

The SH5 test above is **not PhysX**. The separate SH5 USD startup attempt did
not reach READY within the bounded test and is not claimed successful. SH5
link FK, model load, enclosing collision boxes, typed ROS command routing and
watchdogs were verified separately; SH5 dynamics and actual motors remain
unverified. Neither model's test used real Quest tracking or real DXL health.

One initial SG2 test correctly stopped because the usual all-zero-shoulder,
-1.57rad-elbow fixture had only about12.1mm native arm/body collision clearance,
below the existing20mm minimum. The new passive initial-collision check catches
this before activation; the threshold was not weakened.

Physical SG2/SH5 motor commissioning, real Quest latency, external obstacles,
payload behavior and emergency-stop behavior still require on-site verification.
The available controller collision model does not contain arbitrary furniture
or people. No physical robot was automatically moved while preparing this release.

## Rollback

Stop both new publishers/controller/gate. Use the original physical follower and
previous Vuer/Cyclo commands from clean terminals. Original source, URDF, SRDF
and configuration files were not overwritten. Simulation commands remain in
the root `XR_*` documents and use domains231/232, never real domain30.

## SH5 hands with the existing hardware arm path (2026-10-22)

`sh5 --hands` adds optical wrist input to `HardwareXRPublisher` and starts the
unmodified installed ROBOTIS `retargeting_teleop`. It does not use the earlier
simulation publisher or the legacy `controller_pc.sh` arm path. The robot build
adds measured finger positions to collision FK and fixes finger QP velocities at
zero: only the native hand retargeter commands fingers. Other passive joints
remain monitored at the captured posture. SG2 and SH5 now both use the captured physical URDF arm position and velocity
limits. The former SH5-only 0.35 rad/s cap and legacy Cyclo joint4 limit are removed.
Refresh both PC and robot source and restart run_controller.sh to generate a new
profile; old fixed-limit SH5 profiles are not accepted. Arm mapping, capture, resume and Cartesian filtering
remain inherited from the existing hardware publisher.

Extra hand-span heuristics, duplicate short-gap disarm, and hand-node-count
checks were removed. Finite/rigid input decoding, original arm guards and native
retarget joint limits remain. Hand points publish while the existing arm session
is ACTIVE and input/feedback is fresh. There is no new hand command relay or
extra hand trajectory slew limiter. Collision FK uses current measured fingers;
this is not a predictive finger collision filter. Missing hand input sends no
new finger targets; it does not actively cancel the follower's last target.

This integration has not been built or tested by the coding agent at the user's
request. Source wiring is complete; actual Quest/hardware operation is unverified.

### Publish the local change first

Run in the changed checkout on the development PC:

```bash
cd ~/Downloads/external_repos/ai_worker_teleop
git add run_xr_hardware_pc.sh hardware \
  robotis_vuer/robotis_vuer/sh5_optical_hand.py \
  robotis_vuer/robotis_vuer/vr_publisher_sh5_hands_hardware.py
git commit -m "Integrate native SH5 hands with hardware arm teleop"
git push origin robotis-applications
```

### PC installation

In the updated `robotis-applications` branch checkout on the mini PC host:

```bash
cd ~/Downloads/external_repos/ai_worker_teleop
git pull --ff-only origin robotis-applications
bash hardware/install_pc.sh
```

This copies the sources used by the Python launcher into the existing PC
container; a PC colcon rebuild is not required. Existing certificates remain.
The container needs its existing ROS Jazzy, Vuer, numpy/scipy and
`robotis_interfaces` installation, as used by the working SH5 hand simulation.

### Robot installation and build

After pushing, on the robot host (first checkout):

```bash
git clone --branch robotis-applications --single-branch \
  https://github.com/MFIWO/ai_worker_teleop.git ~/ai_worker_teleop_sh5
cd ~/ai_worker_teleop_sh5
bash hardware/install_robot.sh
```

For an existing checkout use `git pull --ff-only origin robotis-applications`
in that directory, then run the installer again. Build inside the existing
`ai_worker` container; this does not launch the controller or move the robot:

```bash
docker exec -it ai_worker bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /root/ros2_ws/install/setup.bash
  export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}
  cd /workspace/ai_worker_xr
  python3 -m hardware.build_sh5_hands
'
```

This requires the existing Cyclo C++ source and build dependencies, installed
`cyclo_motion_controller_ros_py` with `retargeting_teleop`, its `retargeting`
Python library and hand models, and the actual SH5 follower/description package.
A Git clone of this PC/helper repository does not install those robot packages.
The separate executable is built under `/workspace/ai_worker_xr_sh5_hands`;
it does not overwrite the verified SG2 startup executable. If the installed
Cyclo source has changed, the build reports the differing source anchor rather
than modifying the installed source blindly. Use `--rebuild` after source updates.

### Each run: variable IP addresses

Start the actual SH5 follower using its established bringup and ready pose first.
This command uses `--arm`, not SG2's `--start`: it does not move to a ready pose.
Both elbows must already be within the actual physical SH5 URDF limits (the
usual ready angle is approximately -1.57 rad). Stop earlier teleop/hand-test publishers.

PC terminal:

```bash
PC_IP=192.168.5.101       # replace for this network
ROBOT_IP=192.168.5.102    # replace for this network
docker exec -it robotis-applications bash \
  /root/ros2_ws/src/robotis_applications/run_xr_hardware_pc.sh \
  sh5 --arm --hands --server-ip "$PC_IP" --robot-ip "$ROBOT_IP"
```

Second PC terminal (SSH to the robot):

```bash
ROBOT_IP=192.168.5.102    # replace for this network
ssh -t "robotis@$ROBOT_IP" \
  'docker exec -it ai_worker bash /workspace/ai_worker_xr/hardware/run_controller.sh sh5 --hands'
```

Quest: `https://<PC_IP>:8012?ws=wss://<PC_IP>:8012`. Use bare-hand tracking;
release gestures first, then hold one hand's pinch plus the other hand's squeeze
for 3 seconds to start. Release before repeating. If the existing arm startup
reports `WAIT_START`, repeat the start gesture after alignment. While ACTIVE,
the same gesture stops input. PC and robot use the existing domain 30/Zenoh path.
IP values are command arguments, not baked into source. If the PC IP changes,
its Vuer certificate must include the new IP; use the existing certificate setup
before launching. The `--server-ip` check reports a mismatch before starting Vuer.

## 2026-10-22: SH5 software limits and deployment

The latest settings and complete push/clone/offline-rsync/build/start commands are in [SH5_DEPLOY.md](SH5_DEPLOY.md).
Cartesian defaults are now 3 m/s and 15 rad/s; input freshness is 5 s and the hardware hold-to-fault grace is 50 s.
Physical URDF position/velocity limits and collision checks remain. Shared hardware defaults also apply to SG2 hardware mode.
SH5 simulation entrypoints are now included in this checkout, with 2 m/s and 10 rad/s Cartesian defaults.
Runtime testing was not performed for these changes.


### Controller hand mode 1 (2026-10-29)

Use `--input controllers --hand-mode 1` on both PC and robot launchers.
A+X starts both arms without holding either grip. Grip proportionally applies
the original thumb/index/middle pinch; trigger applies the original four-finger
grasp. B alone requests the robot-supervised arm ready pose; after completion,
release A/X and press A+X to re-anchor. B+Y holds arms and hands while permitting
MOBILE base motion and HEAD pan/tilt. Both stick clicks still switch HEAD/MOBILE.
Lift remains ACTIVE-only. Fresh tracking, robot feedback and the gate's fault,
lease and collision checks still apply. Loop recording continues during HOLD.
Mode 2 keeps its preset buttons and per-arm grip clutches.

HX5-D20 tactile frames now carry `values_i16` numeric pressure samples alongside
the original uint8 `raw` bytes (left/right, thumb/index/middle/ring/little,
nine row-major samples per finger, 0–255 raw counts). This supplies numeric data
to the tactile source itself. Robot State pressure mirrors remain for existing
training exports. Restart the robot session and record a new episode to include
these numeric tactile fields; old episodes are not rewritten.
