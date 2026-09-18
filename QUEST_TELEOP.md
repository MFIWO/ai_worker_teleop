# AI Worker Quest teleop PC source

This branch retains upstream ROBOTIS-GIT/robotis_applications history at `8d7fe8b`
and adds the following Quest SG2 deployment changes:

- SG2 X+A activation latch, Y+B stop, tracking timeout, reliable trigger commands.
- `controller_pc.sh`: measured-state transition to the elbow 90-degree ready pose,
  followed by the SG2 VR publisher.
- `hand_original_pc.sh`: the same ready pose followed by the unchanged SH5 publisher.
- `hand_neck_pc.sh`: gravity-aligned optical hand arm poses and tracking-loss guard.
- `log_vr_pc.sh`: read-only arm/goal/tracking diagnostics.

Original `vr_publisher_sh5.py` is unchanged. No controller/hand hybrid mode is added.

Robot patches, full execution order and Loop setup are on the
[main branch](https://github.com/MFIWO/ai_worker_teleop/tree/main).
Clone the two branches into separate directories; they are separate components.
The ready scripts move both robot arms. Start only the follower first, then the PC
script, and start the robot VR controller after READY. Consult main's mode-specific
documents before running them.

The existing container source mount must point at this checkout. Certificates are
not included; generate them for the server IP used by Quest. Install the declared
ROS dependencies, including control_msgs and action_msgs, in the PC container.

## Verification

The five added test modules cover SG2 activation, ready trajectories/action results,
gravity alignment and optical hand tracking recovery. Run ROS tests only on isolated
ROS_DOMAIN_ID=231 with ROS and workspace environments sourced. No physical robot is
needed for these tests; ready-pose action servers are simulated.

## Unified operator terminal

See [OPERATOR_CONSOLE.md](OPERATOR_CONSOLE.md) for hand-mode U pause/resume,
manual base/head/lift control and Loop A/B/C forwarding. Start
`bash operator_console.sh` from the local PC desktop terminal after starting teleop.
