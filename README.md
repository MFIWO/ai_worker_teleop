# AI Worker Quest customizations

## SH5 배포와 실행 (현재 기준)

`robotis-applications`는 SH5 팔·손 Teleop 중심 브랜치입니다. `main`은 같은
Teleop 기반과 VR·Loop 녹화/키보드 연동을 함께 제공합니다.

새 PC 설치, 다른 AI Worker 배포, 업데이트, 두 명령 실행 및 종료 방법:
[ai_worker_hand.txt](docs/ai_worker_hand.txt).

팔·손 Teleop만 실행할 때 PC의 터미널 두 개에서:

```bash
bash teleop.sh robot --pc-ip 192.168.5.2 --robot-ip 192.168.5.101 --loop off --bringup
```

```bash
bash teleop.sh pc --pc-ip 192.168.5.2 --robot-ip 192.168.5.101 --loop off
```

`--bringup`은 follower가 없으면 기존 SH5 초기화 동작을 실행합니다. 양손·몸
추적을 유지하고 한 손 pinch + 반대 손 squeeze 3초로 시작/중단합니다.
종료할 때 두 터미널에서 Ctrl+C를 누르고 `[STOP]` 정리를 기다리세요.
이 세션이 시작한 bringup/router와 초기화 자식 노드도 함께 정리됩니다.
Loop·C 키보드 연동은 두 명령 모두 `--loop vr`로 실행하고 PC의 Loop를 먼저 엽니다.

SH5 two-command startup, continuous VR Loop recording and Loop-focused keyboard
controls: [hardware/VR_LOOP.md](hardware/VR_LOOP.md).

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
