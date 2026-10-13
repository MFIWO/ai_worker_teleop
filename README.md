# AI Worker Quest customizations

See [QUEST_TELEOP.md](QUEST_TELEOP.md) for this branch and the matching robot setup.

## Wrist-delta retargeting (SG2 / SH5)

**[ADDED 2026-10-14]** [SG2 / SH5 hardware commissioning](hardware/README.md)
provides the model-specific, arms-only robot setup, slow wrist-delta controller,
fixed calibration, passthrough alignment guides, robot-side communications
watchdog and passive logs. Hardware starts disarmed and retains conservative
position limits; it does not use the expanded simulation elbow model.

The existing simulation experiments are documented in
[XR_OVERHEAD_PATH.md](XR_OVERHEAD_PATH.md),
[XR_CALIBRATION_OVERLAY.md](XR_CALIBRATION_OVERLAY.md) and
[XR_EXTENSION.md](XR_EXTENSION.md). Old `model:=sg2` / `model:=sh5` launchers
remain unchanged. Do not run two publishers or confuse simulation with hardware.

# robotis_applications

ROS Packages for Robotis Applications

This repository contains the official ROS 2 packages for the ROBOTIS applications.

For usage instructions and demonstrations of the ROBOTIS applications, check out:
  - [Tutorial Videos](https://www.youtube.com/@ROBOTISOpenSourceTeam)

To use the Docker image for running ROS packages with the ROBOTIS applications, visit:
  - [Docker Images](https://hub.docker.com/r/robotis/robotis-applications/tags)

## Docker Usage

The main entrypoint is `docker/container.sh`.
Docker builds for both `amd64` and `arm64` use the shared `docker/Dockerfile`.

Build and start the container:

```bash
./docker/container.sh start
```

On the first run, `start` also generates `robotis_vuer` certificates automatically
if `cert.pem` and `key.pem` are missing, using the host IP detected on the host
machine.

Enter the running container:

```bash
./docker/container.sh enter
```

Stop the container:

```bash
./docker/container.sh stop
```

## Launch VR Publisher

The `robotis_vuer` package provides one launch file with a `model` argument.

Run SH5: (default)

```bash
ros2 launch robotis_vuer vr.launch.py model:=sh5
```
or shortcut:
```bash
vr model:=sh5
```

Run SG2:

```bash
ros2 launch robotis_vuer vr.launch.py model:=sg2
```

or shortcut:
```bash
vr model:=sg2
```

Run HX5:

```bash
ros2 launch robotis_vuer vr.launch.py model:=hx5
```

or shortcut:
```bash
vr model:=hx5
```

## Third-Party Notice

This project uses [`vuer`](https://github.com/vuer-ai/vuer), which is licensed under the MIT License.
