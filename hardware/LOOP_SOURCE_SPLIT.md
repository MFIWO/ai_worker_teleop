# VR Loop recording: unified aiworker state and HX5-D20 tactile

Updated 2026-10-29. Replaces the previous three-source description. This
changes recording, not arm/hand teleoperation, camera transport, or motor
commands. The GitHub change does not update running PC/robot containers.

## Loop Preview sources

| Kind | Source key | Display name | Contents |
|---|---|---|---|
| Robot State | robotis-vr-sh5 | aiworker | Robot and both HX5-D20 hand qpos; selected actions/quality fields |
| Tactile | robotis-vr-sh5-tactile | HX5-D20 | left_hand and right_hand, each 45 raw uint8 fingertip measurements |

There is exactly ONE Robot State source. Do not select the retired
robotis-vr-sh5-hand Robot State source. The tactile source uses
SourceSchema(tactile=TactileStreamSchema(...)) with send_tactile(),
and also mirrors pressure into numeric Robot State channels for Parquet export.
The robotis-vr-sh5 source key is unchanged but its layout changes:
re-select and re-save the Loop cell configuration. Neither sender can
automatically migrate a saved Loop configuration.

Robot qpos and hand qpos are matched by ROS joint names. Missing observations
are fixed-size NaN vectors with observation_valid=0. Both sources use the same
PC-anchored timestamp/tick sequence. If a pressure sample is missing,
invalid, or stale, its Tactile channel is omitted, never zero-filled.
HOLD does not stop recording; fresh/valid action flags matter at export.

## Choose training channels

Set --loop-channels on the ROBOT terminal invocation (not the PC terminal).
This is an SDK schema allowlist, not a post-recording dataset filter:

- training (default): observation + action joint positions for left_arm (7),
  right_arm (7), left_hand (20), right_hand (20), head (2), lift (1),
  both hands' tactile_pressure (45 each) and tactile_status (4 each),
  and teleop.flags, teleop.action_valid and teleop.observation_valid.
  19 channels total with tactile enabled; 15 with --loop-no-tactile.
  Mobile velocity and all other diagnostics are excluded.
- qpos: six observation joint_position channels for arms, hands, head and lift,
  without actions/teleop/velocity.
- all: all mapped joint obs/action channels, mobile velocity, teleop metadata
  and numeric pressure/status. Typed Tactile remains available separately.
- Custom: an exact comma-separated list of keys, either with or without
  the ffw_sh5_vr. prefix. Unknown/duplicate/empty keys fail at startup.
  Example: observation.left_arm.joint_position,observation.left_hand.joint_position,teleop.flags

The six joint groups are left_arm, right_arm, left_hand, right_hand, head,
and lift. The available keys are in these forms:

    ffw_sh5_vr.observation.GROUP.joint_position
    ffw_sh5_vr.action.GROUP.joint_position

Mobile velocity keys (only under all or custom) are
ffw_sh5_vr.observation.mobile.velocity and
ffw_sh5_vr.action.mobile.velocity.

Each joint group uses radians except lift (meters). For behavior cloning,
retain teleop.action_valid: its seven entries correspond to left_arm,
right_arm, left_hand, right_hand, head, lift, mobile. Cached actions may
are replaced with measured hold positions during HOLD; exclude invalid action
frames from training. Every numeric channel remains present from the first
tick, preventing the observed late lift-action `parquet columns changed` error.
teleop.flags is [active,paused,manual_active,fault,status_fresh].
The observation_valid mask has the same seven-group order. If you choose only
raw qpos, you intentionally lose the validity mask and numeric pressure.

--loop-no-tactile disables registration and subscriptions for the separate
Tactile source and numeric pressure mirror, but keeps hand qpos inside aiworker. --loop-no-camera is
independent. Neither changes motor control, follower setup or ROS feedback.

## Commands on PC host (replace PC_IP and ROBOT_IP)

Default training and typed Tactile, robot terminal:

~~~bash
bash ~/Downloads/external_repos/ai_worker_teleop/teleop.sh robot \
  --pc-ip PC_IP --robot-ip ROBOT_IP --loop vr \
  --input hands --hand-mode 1 --bringup
~~~

Robot + hand qpos only, omit tactile:

~~~bash
bash ~/Downloads/external_repos/ai_worker_teleop/teleop.sh robot \
  --pc-ip PC_IP --robot-ip ROBOT_IP --loop vr \
  --input hands --hand-mode 1 --bringup \
  --loop-channels qpos --loop-no-tactile
~~~

Choose exact channels:

~~~bash
bash ~/Downloads/external_repos/ai_worker_teleop/teleop.sh robot \
  --pc-ip PC_IP --robot-ip ROBOT_IP --loop vr \
  --input controllers --hand-mode 2 --bringup \
  --loop-channels 'observation.left_arm.joint_position,observation.right_arm.joint_position,observation.left_hand.joint_position,observation.right_hand.joint_position,action.left_arm.joint_position,action.right_arm.joint_position,action.left_hand.joint_position,action.right_hand.joint_position,teleop.flags,teleop.action_valid'
~~~

The PC terminal command is unchanged: teleop.sh pc --pc-ip PC_IP
--robot-ip ROBOT_IP --loop vr --input hands|controllers.

Tactile subscriptions use /left_hand/finger_pressures and
/right_hand/finger_pressures (robotis_interfaces/msg/HandPressures).
An absent ROS message type or topic produces a warning rather than fake
readings. Verify with ros2 topic echo inside the robot's running environment.

## Offline checks and limitations

~~~bash
python3 -m unittest hardware.test_loop_split hardware.test_loop_vr
~~~

The real SDK schema test is skipped when loop_sdk is not installed;
run inside the ai_worker container to check installed SDK classes.
The optional isolated ROS test is hardware.test_loop_vr_runtime with
ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp.
These tests use mock sinks and do not command motors or create Loop recordings.
The ROS test also round-trips every frame through the installed SDK protobuf
codec and verifies fixed columns through startup, HOLD, late lift commands,
pressure arrival and pressure timeout.
A physical Preview source-type check and actual short saved episode inspection
are still required before trusting training data.
