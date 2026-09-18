# ffw_loop_streamer — Config Loop streamer for the ROBOTIS AI Worker

ROS 2 (Jazzy) sidecar that runs **inside the `ai_worker` container** next to `ffw_bringup`
and streams the robot's state/action and cameras into **Config Loop v1.1.4** via
`loop-sdk` 0.4.1. Read-only: it subscribes to the topics `ffw_sg2_ai` produces and never
publishes to the robot.

```
ffw_bringup (ffw_sg2_ai)                     ffw_loop_streamer (node: loop_streamer)
  follower /joint_states, /odom ─────────┐
  leader   /leader/*/joint_trajectory, ──┤  robot-step (gRPC :50051) ──▶ Config Loop
           /cmd_vel                      │  camera-* (RTSP 8554..8556) ◀── Loop pulls
  cameras  .../compressed ───────────────┘
```

Channel layout per robot config (`config/ffw_sg2_rev1.yaml`, copied from
physical_ai_tools so vectors match LeRobot datasets): `<robot>.observation|action.<group>.joint_position`
for `left` (7 arm joints + gripper = 8), `right` (8), `head` (2), `lift` (1) and
`<robot>.observation|action.mobile.velocity` (3). Grippers are element 8 of the arm vectors,
exactly as ai_worker groups them.

## Install (robot, offline)
See `../loop_rollout/ROLLOUT.md`. Summary: copy this directory to `~/ai_worker/ffw_loop_streamer`,
`pip install --no-index` the aarch64/cp312 wheels for `loop-sdk==0.4.1` into the container's
`/opt/venv` (protobuf 6 → 7; only onnx depends on it there), `cb` (colcon build), then persist with
`docker commit` → `robotis/ai-worker:1.2.1-loop` and `docker/.env` (`LOOP_ADDR`, `LOOP_ROBOT_TYPE`).

## Run (inside the ai_worker container)
```bash
ffw_sg2_ai_loop     # = ros2 launch ffw_loop_streamer ffw_sg2_ai_loop.launch.py  (follower+leader+cameras, streamer after 25 s)
loop_streamer       # = ros2 launch ffw_loop_streamer loop_streamer.launch.py   (streamer only, next to a running ffw_sg2_ai)
#   args: robot_type:=ffw_sg2_rev1 loop_addr:=<loop-host>:50051 fps:=30.0 enable_camera:=true camera_probe_timeout:=120
```
`loop_addr` defaults to `$LOOP_ADDR` (set via `docker/.env`, applied when the container is
(re)created). `fps` must equal the Loop cell-config Hz or video and robot-step drift apart.
When launching by hand: `export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH`
(the console script's shebang is `/usr/bin/python3`).

## Tests
`cd ~/ai_worker/ffw_loop_streamer && python3 -m pytest -q test/test_loop_streamer.py` (17 offline tests).
Smoke test without ROS: `python3 -m ffw_loop_streamer.loop_streamer --loop-addr <loop-host>:50051`.

## Notes
- Cameras: the node waits `camera_probe_timeout` (120 s) for the first frame of each camera, then
  streams without the missing ones for the rest of its life. Start cameras first or restart the streamer.
- RTSP server speaks RTP/AVP/TCP (interleaved) only; `ffplay -rtsp_transport tcp rtsp://<robot>:8554/cam_head`.
- Adding a hand (e.g. Inspire): add `hand_left`/`hand_right` entries to `joint_topic_list`, `joint_list`
  and `joint_order` in the config; channels appear automatically.

## VR arm recording

See [VR_LOOP.md](../VR_LOOP.md) for the separate `ffw_sg2_vr` profile: seven arm
joints per side plus independent gripper channels, using final VR controller
commands as actions and `/joint_states` as observations. Use `source_key:=robotis-vr`.
