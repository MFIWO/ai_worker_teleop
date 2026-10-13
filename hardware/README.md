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
Communication failures, missing/replayed health timestamps, missing torque
evidence, hardware errors or a PC lease older than0.5s latch the gate. The
upstream driver's published `torque_state` may remain False while torque is
actually ON; this release therefore **does not treat False as a warning alone**.
It requires fresh, independent, positive `Torque Enable` register reads for
all 14 physical arm motors via `/ffw_follower/get_dxl_data` before accepting
that mismatch. A failed, missing or stale register read continues to block
motion. No component in this release turns torque on automatically. Fresh
joint-state timestamps alone are insufficient; the hardware driver may publish
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

## SG2 three-terminal hardware startup (follower → PC ready → Cyclo)

**One-time setup:** PC and robot clocks must be synchronized (within the
existing 0.5s message-age limit). The user's previous robot clock was over
40 hours behind the PC; do not disable the timestamp checks. Confirm that the
robot's original physical emergency stop works and the workspace is clear.

The SG2 native home `arm_[lr]_joint4=0` is outside Cyclo's native
joint4 upper bound `-0.7rad`. This is WHY starting `run_controller.sh sg2`
before the existing measured `-1.57rad` ready pose failed. The intended
order is the same as the previously working `controller_pc.sh` path.

**Robot terminal 1: existing physical follower** (exactly once, in ai_worker Docker):

```bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```

**PC terminal 2: after follower is stable, ready pose then Vuer** (in
robotis-applications Docker; replace the IP with the actual robot router):

```bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp
unset ZENOH_CONFIG_OVERRIDE
bash /root/ros2_ws/src/robotis_applications/run_xr_hardware_pc.sh \
  sg2 --ready-pose --arm --robot-ip 192.168.5.102
```

PC first performs a **read-only** bus/torque/clock precheck. If the upstream
`torque_state` reports False, it reads each physical arm's `Torque Enable`
register; only **all fourteen registers=1** plus a healthy bus can continue.
If some are truly OFF, no motion is sent: use the manufacturer's verified
follower procedure and never bypass the check. The existing
`prepare_sg2_ready_pose.py` then sends a measured-start, five-second-minimum
quintic path to both 7-joint arm action controllers. It requires action
success and measured convergence before starting Vuer. Old VR/Cyclo writers
must not run during the move. Vuer starts inactive until manual X+A.

**Robot terminal 3: start only after PC displays `READY`** (ai_worker Docker):

```bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp
bash /workspace/ai_worker_xr/hardware/run_controller.sh sg2
```

The robot passively snapshots the **new measured ready posture** and checks
native joint bounds, self-collision clearance, live motor/clock feedback and
model consistency. It launches Cyclo, checker and gate inactive. The gate
retains the original speed bounds, robot/PC freshness and fault latches.
Connect Quest to Vuer; release X/A before a fresh X+A after READY.
Y+B stops tracking. No hand/gripper/base/lift target is forwarded.

You may run `bash .../run_controller.sh sg2 --check-only` after the ready
pose to test without launching Cyclo. `--allow-torque-off` is **passive
check-only** and never permission to move the real robot.

**SH5:** the SG2 automatic ready mover does not apply. The existing SH5
follower/model snapshot procedure still applies. Do not use `--ready-pose`
with SH5.

## Robot terminal — existing SG2/SH5 model details

The controller wrapper never starts the follower or issues a ready pose. It
generates a unique robot-local profile after stable measured feedback, checks
the native geometry and joint limits, and starts inactive. The existing
Cyclo/CBF/reference-checker and hardware gate remain active guards.
Generated profiles are saved under
`/workspace/ai_worker_xr_profiles/{model}_<unique timestamp>/`.
A failed native clearance or a joint outside conservative limits is a
reason to stop, not to widen model limits.

The SH5 reduced-control-model handling and frozen-joint restrictions described
above continue to apply. This flow has synthetic integration evidence, not a
completed real-motor deployment test.

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
