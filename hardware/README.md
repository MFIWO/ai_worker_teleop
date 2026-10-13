# SG2 / SH5 arms-only hardware commissioning

Added 2026-10-14 on the existing `robotis-applications` branch. This is an
**opt-in hardware setup**, not a claim of a completed physical robot test.
The original SG2/SH5 publishers and all simulation modes remain available.

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
| Wrist goal speed / angular speed | 0.06m/s / 0.35rad/s |
| Arm joint velocity bound | at most0.35rad/s |
| Joint position bounds | installed conservative Cyclo bounds, no expansion |
| SG2/SH5 elbow joint4 upper in the checked source | -0.7rad, **not** simulation -0.45rad |
| Measured geometric workspace | original0.94 reach fraction, not simulation0.995 |
| Wrist position/orientation weights | existing10/1 |
| Elbow auxiliary cost | 0; wrist-centric7-DoF solve |
| QP / collision / CBF / reference checker | original installed implementation retained |
| Hand / gripper / trigger / base / lift commands | none forwarded; exactly7arm joints per side |

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
Communication failures, missing/replayed health timestamps, arm torque off,
hardware errors or a PC lease older than0.5s latch the gate. Fresh joint-state
timestamps alone are not enough: the hardware driver can temporarily publish
cached joint values during a motor-read failure.

Cyclo receives a canonical-order joint-state stream from the gate because its
installed callback caches name indexes from the first message. The gate never
renames a missing joint or invents a measured position.

After a gate fault, inspect the cause and restart the gate/controller and PC;
X+A does not clear the fault. Normal Y+B stop/resume retains the calibration.

**This software gate is not a certified emergency stop.** A process hard-kill,
host power loss or motor-driver failure is not covered by a Python stop callback.
Use the physical emergency stop and the normal hardware driver's watchdogs.
ROS node names/domain IDs are not authentication: keep the robot network trusted
and isolated from unrelated command publishers. Do not run two robots in the
same un-namespaced domain30 graph. PC and robot clocks must be synchronized.

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
It generates a new profile, verifies collision mesh resources and checks native
clearance for collision pairs affected by arm movement before starting Cyclo.
The gate and controller start **inactive** until a checked manual X+A.

For a **passive-only** check, add `--check-only`. No Cyclo or gate is launched.
If the measured initial pose is outside original limits or lacks native
collision clearance, it refuses rather than moving automatically. Use the
robot's known-good ready-pose workflow, keep clear, then retry. Do not change
the clearance threshold to make an unverified pose pass.

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
