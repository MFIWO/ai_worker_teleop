# SG2 passthrough calibration guide (display only)

Added 2026-10-14. The guided publisher subclasses the existing
`XRExtensionPublisher`. It changes only the Vuer scene, not calibration,
retargeting, startup, controller launch, solver precision, or safety thresholds.
The previous publishers and launchers remain unchanged. Nothing is pushed.

## Run

Keep the existing SG2 Isaac Sim and AI Worker extension-controller commands.
Stop the existing Vuer publisher, then run **inside ROBOTIS Applications**:

```bash
export ROS_DOMAIN_ID=231
bash /root/ros2_ws/src/robotis_applications/run_sg2_xr_guided_sim.sh
```

Refresh the Quest browser and enter the same passthrough XR session as before.
The ROBOTIS guide calls the button **Enter VR** even though passthrough is
shown. Do not open a second Vuer/control session. Existing port8012 and
certificates are reused. No new runtime dependency is installed.

Official workflow reference:
https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/vr_teleoperation/

## What is displayed

- White point + short RGB axes: current tracked controller origin and rotation.
- Cyan L / orange R wireframe target + long RGB axes: controller pose required
  to match the held startup goal or the currently measured simulated wrist.
- Green target: the pose-alignment check is met (20mm robot-frame position,
  10deg orientation, no radial clipping), **not** permission to bypass any
  other controller check. Both held and measured alignment are considered
  during stationary startup.
- Connecting line: the position correction from the live point to the target.
- Text panel: WARMUP/ramp remaining, WAIT_START, ALIGN_REQUIRED, or ACTIVE.
- Missing tracking/TF/joint feedback: target markers are hidden, not left green.
  A complete browser/server disconnection may freeze the browser's last frame;
  that is not a valid alignment/activation indication.

Align the **white tracked point** with the target, not an assumed centre of
the physical controller/palm. Match the axes as well: a position match alone
is insufficient. The thin and thick axes share X-red, Y-green, Z-blue colours.
Targets are in WebXR Y-up metres, not ROS base_link coordinates. Only text
billboards/the status panel face or follow the viewer; targets do not follow
the current head or hand pose.

Before the **first** X+A there is no uniquely defined target: this is a delta
mapper. Choose a comfortable pose and calibrate; the view initially shows
only the live origins and instructions. After capture, the guide shows where
to hold/return. It does not invent a default operator arm length or home pose.

During WARMUP, wait for READY. If WAIT_START appears, align both poses, release
X/A, then press X+A. The existing implementation decides whether to start.
Y+B remains stop. Later X+A uses the original session, never a new hand zero.
Restart/reset/recenter of the WebXR reference space must not be mistaken for
the same calibration; stop and follow the existing new-session procedure.

## Why the target is correct

The original mapping is

```
p_r = p_r0 + scale * B * (p_c - p_c0)
R_r = B * R_c * R_c0.T * B.T * R_r0
```

The guide uses the inverse, without modifying any of the original anchors:

```
p_c_target = p_c0 + B.T * (p_r_measured_or_held - p_r0) / scale
R_c_target = B.T * R_r_measured_or_held * R_r0.T * B * R_c0
```

On resume, the target is not necessarily the very first neutral hand pose:
the robot may have stopped elsewhere. It is the controller pose corresponding
to that stopped robot pose under the **original** mapping. At scale !=1,
the displayed position tolerance sphere is 0.02/scale metres; the robot-side
tolerance remains 0.02 metres. The display never calls `_capture`, `_publish_pair`,
or `_set_arm_control_enabled` to change control state.

## Files

```
robotis_vuer/robotis_vuer/xr_calibration_overlay.py
robotis_vuer/robotis_vuer/vr_publisher_sg2_xr_guided.py
robotis_vuer/test/test_xr_calibration_overlay.py
run_sg2_xr_guided_sim.sh
```

Tested with the already installed Vuer0.1.6. Fifteen new tests cover
nonidentity rotation and scale inversions (160 bilateral random cases),
head/hand-independent targets, measured-versus-held alignment, orientation,
stale markers, first calibration, session immutability, native scene
serialization, and unchanged control-method inheritance. Combined XR tests
77/77 and existing shoulder-profile tests4/4 passed. The actual scene
coroutine was exercised with captured SET/UPSERT messages. A separate
UI-only Vuer server on loopback8013 (no ROS initialization) also delivered
SET and three UPSERT packets through an actual WSS client, including
removing target markers on stale feedback and restoring them on recovery.
The probe used Vuer's normal INIT handshake; it did not test certificate trust
or any headset render. Packet capture is in the ROBOTIS Applications mount:
`docker/workspace/calibration_overlay/wss_packets.json`.

Evidence: `~/Downloads/.retarget-review/calibration_overlay_20261014/`.
**Actual Quest passthrough rendering, label readability and visual origin
alignment still need a headset test.** Passing Python/schema tests is not
a headset-render test. No robot/simulator motion was started for this change.

Rollback: stop guided Vuer and use the unchanged `run_sg2_xr_extension_sim.sh`.
The original passive logger command remains valid; its session/phase fields
continue to come from the original extension publisher.
