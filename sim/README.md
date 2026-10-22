# SG2 Isaac Sim DDS bringup — Quest arm retargeting

**2026-10-13 [ADDED].** This is an SG2-specific adaptation of
`sh5_dds_bringup.py`. It only replaces physical joint command/feedback;
the existing Vuer, arm retargeter and Cyclo VR controller remain responsible
for pose processing, QP/IK and reference checks. It **does not control a real
robot** and **does not** launch a physical follower.

## What was measured

- `FFW_SG2_CFG` / `FFW_SG2.usd` was instantiated in Isaac Sim 5.1.
- DDS domain 231 accepted a left-arm
  `JointTrajectory`. PhysX measured `arm_l_joint4` changing from -1.57
  to -1.25 rad. `/joint_states` and `/tf` were received with
  `arm_l_link2` included. No Quest/WebXR or actual ROS Cyclo controller
  was involved in this isolated test.
- The ROS2 Jazzy Fast DDS client inside `ai_worker` Docker also received
  the simulator's `/joint_states` using **default network discovery**.
  With `CYCLONEDDS_URI` restricted to loopback and
  `ROS_LOCALHOST_ONLY=1`, the Fast DDS client timed out. Therefore
  **do not combine those loopback-only settings** for this integration;
  isolate from the actual robot by Domain 231 and network control instead.
- A **real ROS2 Jazzy Fast DDS publisher** inside the `ai_worker` Docker
  published `trajectory_msgs/msg/JointTrajectory` to the existing left
  arm command topic. The host CycloneDDS SG2 bridge received the command,
  moved the simulated `arm_l_joint4` from -1.57 to -1.30 rad, and delivered
  221 joint-state and 222 TF messages during the simultaneous probe.
  This verifies ROS2 → DDS → PhysX → DDS feedback, not Quest pose tracking.
- A bounded **GUI launch** on the user's X11 `DISPLAY=:1` also reached
  `[READY] SG2 DDS simulation`. Isaac Sim emitted existing USD
  material-binding scope warnings; they did not prevent bridge startup.
- The SG2 USD's `base_link` is a fixed child of the rigid `world` link:
  the bridge composes the authored offset with measured rigid world pose
  before publishing `base_link -> arm_..._link` transforms.
- A controller trajectory may include `gripper_[lr]_joint1` for the Quest
  trigger. The remaining gripper-link coupling is **not yet empirically
  validated**. Wheel/swerve commands are never subscribed to.

## Local host Python environment

The PC already has Isaac Sim 5.1 and Isaac Lab in `groot-t1`. The
implementation was tested with a separate Python venv and native DDS library
under `/home/kimm/Downloads/.sg2-bridge-runtime`, leaving `groot-t1`
unchanged:

```bash
cd ~/Downloads/external_repos/cyclo_lab
conda activate groot-t1
export SG2_RUNTIME="$HOME/Downloads/.sg2-bridge-runtime"
export PYTHONPATH="$PWD/source/cyclo_lab:${PYTHONPATH:-}"
export CYCLONEDDS_HOME="$SG2_RUNTIME/cdds"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=231
# [FIX] Fast DDS <-> CycloneDDS requires default discovery on this PC.
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
"$SG2_RUNTIME/venv/bin/python" scripts/sim2real/bringup/sg2_dds_bringup.py --check-only
```

## Terminal A — Isaac Sim SG2

```bash
cd ~/Downloads/external_repos/cyclo_lab
conda activate groot-t1
export SG2_RUNTIME="$HOME/Downloads/.sg2-bridge-runtime"
export PYTHONPATH="$PWD/source/cyclo_lab:${PYTHONPATH:-}"
export CYCLONEDDS_HOME="$SG2_RUNTIME/cdds"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=231
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
"$SG2_RUNTIME/venv/bin/python" -u scripts/sim2real/bringup/sg2_dds_bringup.py --domain_id 231
```

Wait for `[READY] SG2 DDS simulation`. Add `--headless` to skip GUI.
Do **not** start `sh5_dds_bringup.py` concurrently.

Use another host terminal to check **simulation only** DDS:

```bash
cd ~/Downloads/external_repos/cyclo_lab
export SG2_RUNTIME="$HOME/Downloads/.sg2-bridge-runtime"
export CYCLONEDDS_HOME="$SG2_RUNTIME/cdds"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
"$SG2_RUNTIME/venv/bin/python" scripts/sim2real/bringup/probe_sg2_dds.py --domain_id 231
```

The probe is read-only by default. Use `--move-left-elbow -1.30` ONLY
when the simulator is running and the physical robot is isolated. The
probe refuses motion in domain 30.

To check ROS2 subscriber connectivity from `ai_worker` Docker after
Terminal A reports `[READY]` (read-only):

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=231
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
ros2 topic echo --once /joint_states sensor_msgs/msg/JointState
```

## Terminal B — AI Worker Docker (after corrected retarget installation)

```bash
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=231
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
```

**[DEBUG]** As checked on 2026-10-13, the running `ai_worker` container's
installed retarget launch did **not** have the new
`arm_input_frame`/geometry arguments. Do not launch the old retargeter
with new `human_arm_level` inputs. Deploy the reviewed
`ai_worker_teleop` corrected Python/launch pair into an **isolated overlay
workspace**, source that workspace, then launch:

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
    controller_type:=vr hand:=false \
    arm_input_frame:=human_arm_level arm_wrist_distance_priority:=0.0
```

Do not start an AI Worker physical follower. A functioning controller also
needs the SG2 measured `/joint_states` and `/tf` from Terminal A. Any
reference/startup mismatch must be diagnosed, never bypassed.

## Terminal C — ROBOTIS Applications Docker

Current `robotis-applications` container mounts
`~/Downloads/external_repos/ai_worker_teleop_pc` directly, including the
locally corrected `vr_publisher_sg2_neck.py`. The corrected optical
frame publisher is **not** selected by `model:=sh5` or by
`model:=sg2` without modification.

```bash
docker exec -it robotis-applications bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=231
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
export PYTHONPATH=/root/ros2_ws/src/robotis_applications/robotis_vuer:${PYTHONPATH:-}
python3 /root/ros2_ws/src/robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2_neck.py
```

**[SAFETY]** Do **not** use `controller_neck_pc.sh` or
`controller_pc.sh` for simulation-only operation: they request physical
arms to move into the ready position.

## Domain 30 and verification limits

Domain `231` is a chosen simulation domain; all three processes
must use the same Domain and compatible transport/discovery settings.
Default discovery may reach other machines on the network, so verify
that the physical SG2 (normally domain 30) and other actors are
not using domain 231. For additional safety, disconnect the physical
robot's network when experimenting with motor-like commands. `30` can also
work if the physical robot DDS network is *fully* isolated. For this reason,
the SG2 bridge requires `--allow_domain_30` whenever `--domain_id 30`
is specified. Merely disabling the robot VR controller is not isolation.

Live Quest through the AI Worker controller into this new SG2 bridge has
**not been verified**. Only the actual CycloneDDS-to-PhysX-to-CycloneDDS
isolated loopback probe passed. Preserve all startup/reference and
tracking-loss safety checks while integrating.
