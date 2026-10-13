# SG2 XR: shoulder-free task profile

2026-10-13. **[ADDED] Opt-in controller configuration only.** The working
XR Delta publisher, calibrated wrist mapping, geometric workspace guards,
Isaac Sim bridge, original Cyclo config/C++, and URDF/SRDF are unchanged.

## Cause and change

The three shoulder joints were not locked. All 14 arm joints participate in
the installed Cyclo QP. At neutral, SG2 joint1 rotates about local +Y
(pitch), joint2 about local +X (roll), joint3 about local +Z (yaw/upper-arm
axial rotation). These are serial joint axes, not three independent global
Euler angles. Left joint2 is bounded at [0,3.14], right at [-3.14,0].

The preceding XR mode holds the previous geometric elbow bend plane. The
installed controller's elbow position weight is 8, versus wrist position
10 and orientation 1. This makes elbow movement expensive and can suppress
shoulder roll. Dropping subgoal messages is not sufficient: without an
elbow sample the controller still penalizes zero elbow velocity at weight8.

The new profile clones the **entire currently installed config**, changing
exactly one field:

```yaml
vr_controller:
  ros__parameters:
    weight_elbow_position: 0.0  # [CHANGED] was 8.0
```

This snippet is explanatory, NOT a complete replacement YAML. The generated
file contains every original setting including damping0.1, wrist weights,
startup checks, reference checker, collision parameters and topic names.
The original installed config SHA256 when generated was
`d9d09b3d5e9189c20433bf6f1b485edb0e10d930fe5a3d266a20bb1809e1b599`.

The original C++ caches weights in its constructor. Restart with the new
config; `ros2 param set` alone does not update the cached QP weight.

## Run: only the AI Worker launch changes

Keep the existing SG2 Isaac Sim command and network settings. Stop the old
XR publisher and VR launch before switching profiles; do not run duplicates.
In the **AI Worker Docker** terminal with its ROS/workspace sourced:

```bash
export ROS_DOMAIN_ID=231
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS

ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
  controller_type:=vr hand:=false arm:=false \
  config_file:=/workspace/sg2_xr_profiles/shoulder_free.yaml
```

That file was created in the current container's persistent `/workspace`
mount. In **ROBOTIS Applications Docker**, use the same XR command as before:

```bash
export ROS_DOMAIN_ID=231
bash /root/ros2_ws/src/robotis_applications/run_sg2_xr_sim.sh
```

Manual X+A calibration, Y+B stop, waiting for `Controller activated`, and
the existing slow-start remain unchanged. The optional verification command
in another identically sourced/domain-matched AI Worker terminal is:

```bash
ros2 param get /vr_controller weight_elbow_position
# Expected: Double value is: 0.0
```

To roll back, stop the publisher/controller and remove only
`config_file:=/workspace/sg2_xr_profiles/shoulder_free.yaml` from the launch.
The old installed config still has elbow weight8. `arm:=false` stays because
both profiles use XR Delta directly, not the legacy Python retargeter.

## Actual comparison on this PC

Both runs used real ROS2, the installed Cyclo C++ QP, and SG2 Isaac Sim PhysX
in domain232. No physical robot or live Quest was used. The same test-only
FK fixture generated wrist references corresponding to a modest pitch,
then roll, then yaw arm movement. **Only wrist poses were sent into the
real XR callbacks**; no desired joint angles were sent to the simulator.

The observed joint ranges during the fixture were:

| Actual left shoulder joint | Existing elbow8 | New elbow0 |
|---|---:|---:|
| joint1 / pitch | 38.89 degrees | 22.92 degrees |
| joint2 / roll | 3.30 degrees | 21.86 degrees |
| joint3 / yaw | 55.17 degrees | 34.37 degrees |

Right-arm ranges matched to the shown precision. The result is **not** that
all shoulder axes should move more: removing the elbow objective changed
the joint distribution, notably permitting more roll instead of extra
pitch/yaw compensation. Wrist position/orientation checks passed in both
runs. The new run remained active throughout the motion and Y+B stopped it.
Each run's startup, activation delay and slow-start remained enabled.

End-of-fixture wrist error in this synthetic test was about3.22mm/0.25deg
for elbow8 and below0.01mm/0.01deg for elbow0. This is a settled,
model-generated, simulated fixture, not evidence of live Quest accuracy or
real hardware precision. Do not extrapolate to arbitrary trajectories.

Raw time series and logs:

```text
docker/workspace/shoulder/original.json
docker/workspace/shoulder/free.json
/home/kimm/Downloads/.retarget-review/shoulder/original/
/home/kimm/Downloads/.retarget-review/shoulder/free/
```

Original15 math +5 real-rclpy safety tests and new4 config preservation tests
passed. The profile's parsed YAML was compared to the installed original;
exactly the elbow weight differed, and the source hash was unchanged.

## Limits and interpretation

This enables shoulder participation in **wrist-centric teleoperation**. It
does not measure or directly copy human shoulder angles. When wrist position
and orientation stay identical while a person swivels only the elbow, those
wrist inputs contain no unique command for the extra arm degree of freedom.
Reproducing that motion needs an additional reliable elbow/posture input.
H1_2/H2's general wrist-centric idea is relevant; its IK solver, regularizers,
robot dimensions and offsets were not copied into SG2.

The geometric radial workspace shell and clutch/speed/freshness/reference
guards remain in the XR publisher, but the generated elbow messages now have
zero QP weight. Near that shell or singular/limit configurations, the original
guard can still stop; this change is not a workspace expansion. QP damping
regularizes velocity, not a specified shoulder posture. Redundancy therefore
remains path-dependent. Collision and joint-limit settings were preserved;
their unchanged slack-based QP is not a hard physical-safety guarantee.

## Recreate the config on another setup

The repository contains `sim/make_shoulder_free_config.py`. Copy that small
script into a writable directory in AI Worker Docker, source its installed
ROS/workspace, and run it with a new output path:

```bash
python3 make_shoulder_free_config.py --output /workspace/sg2_xr_profiles/shoulder_free.yaml
```

It resolves the installed package config automatically, refuses to overwrite
that config or a different existing output, and reports the source SHA256.
No package upgrade, C++ rebuild or Isaac Sim reinstall is required.
