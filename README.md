# AI Worker SH5 VR · Loop integration

이 `main` 브랜치는 **SH5 팔·손 통합 VR Teleop과 Loop 녹화·키보드 연동**을
다른 PC/AI Worker에 함께 배포하는 통합판입니다. `robotis-applications` 브랜치는
같은 기반의 팔·손 Teleop 중심 안내를 제공합니다. Main에도 PC 컨테이너 구성과
Teleop 코드가 포함되므로 새 PC에는 이 브랜치 하나를 clone하면 됩니다.

## 설치와 업데이트

```bash
mkdir -p ~/Downloads/external_repos
cd ~/Downloads/external_repos
git clone --branch main --single-branch \
  https://github.com/MFIWO/ai_worker_teleop.git ai_worker_teleop_deploy
cd ai_worker_teleop_deploy
```

Docker Engine/Compose v2를 준비한 PC에서:

```bash
bash docker/container.sh start
bash hardware/install_pc.sh
```

PC IP 인증서 생성, 로봇으로 rsync, 소스 설치와 SH5 별도 빌드까지의 전체 명령:
[ai_worker_hand.txt](docs/ai_worker_hand.txt).

이미 받은 PC는 해당 폴더에서 `git pull --ff-only origin main` 후 같은 문서의
PC 설치와 로봇 배포 단계를 반복하세요. 로봇에 인터넷이 없어도 PC에서 rsync로
적용할 수 있습니다. 설치는 기존 ROS Jazzy·Cyclo·SH5 follower/손 retarget 환경을
사용합니다. Loop 모드에는 로봇의 `ffw_loop_streamer`·`loop_sdk`와 PC Loop가 필요합니다.

## 실행 — PC 터미널 두 개

현재 주소 예시는 PC `192.168.5.2`, 로봇 `192.168.5.101`입니다.
**PC에서 Loop 앱을 먼저 열고** 실제 장비 주소에 맞춰 실행하세요.

터미널 1:

```bash
bash ~/Downloads/external_repos/ai_worker_teleop_deploy/teleop.sh robot \
  --pc-ip 192.168.5.2 --robot-ip 192.168.5.101 --loop vr --bringup
```

터미널 2:

```bash
bash ~/Downloads/external_repos/ai_worker_teleop_deploy/teleop.sh pc \
  --pc-ip 192.168.5.2 --robot-ip 192.168.5.101 --loop vr
```

`--bringup`은 follower가 없으면 기존 SH5 초기화 동작을 실행합니다. PC를 먼저
실행해도 로봇의 Zenoh 포트를 최대 120초 기다립니다. Quest 접속 주소:

```text
https://192.168.5.2:8012?ws=wss://192.168.5.2:8012
```

Loop에서 source `robotis-vr-sh5` / robot type `ffw_sh5_vr`를 선택합니다.
녹화 시작/저장은 Loop에서 별도로 조작합니다.

## Loop 창의 키보드 조작

| 키 | 동작 |
|---|---|
| C | 팔·손 시작 / HOLD / 재개 |
| WASD | mobile 전후좌우 |
| O / P | 왼쪽 / 오른쪽 회전 |
| 방향키 | 목 tilt / pan |
| U / I | lift 상승 / 하강: leader와 같은 실측 위치 ±10mm jog, 100Hz |
| Shift+A 또는 UI | Loop 녹화 시작; 검토 화면에서는 Loop의 기존 기능 |
| V 짧게 / 길게 | Loop 검토 화면에서 실패 저장 / 폐기 |

C HOLD에서도 같은 에피소드의 영상·데이터는 계속 기록하며 `paused`와
`action_valid` 상태가 남습니다. V는 Loop의 기존 C 기능을 전달하므로 UI의
C/Hold C 안내 문구는 그대로입니다. Loop 창에 포커스가 있어야 키가 작동합니다.

베이스를 움직이면 팔은 HOLD되고 키보드 제어는 계속 사용할 수 있습니다.
이동을 마친 뒤 C로 프로필 갱신을 요청하고, `[BASE RECAPTURED]` 이후 키를 놓고
C를 다시 눌러 팔을 재개하세요. 갱신 중에는 키보드 동작이 잠시 차단됩니다.

## 종료

두 Teleop 터미널에서 Ctrl+C를 누르고 robot session의 `[STOP]` 정리를 기다립니다.
이 세션이 시작한 follower/router와 launch 자식·손 초기화 executor도 정리합니다.
이미 외부에서 실행되어 재사용한 서비스는 소유 대상이 아닙니다. 이전 버전이
남긴 프로세스의 확인·일회 정리 방법도 [배포 안내](docs/ai_worker_hand.txt)에 있습니다.

로봇 host에는 `ros2`가 없을 수 있습니다. `docker exec ai_worker ...`로 들어가
`/opt/ros/jazzy/setup.bash`, `/root/ros2_ws/install/setup.bash`를 source해야 합니다.

## 구성과 측정

- [VR Loop 설계·기록 채널·팔 계산 주기](hardware/VR_LOOP.md)
- 별도 SH5 sparse QP 빌드는 원본 Cyclo와 underlay를 덮어쓰지 않습니다.
- 저장 자세를 사용한 격리 측정에서 팔 출력은 기존 약 17.4Hz에서 100Hz로 개선됐습니다.
  실제 VR·카메라 부하에서의 전체 지연을 보장하는 수치는 아닙니다.
- 기존 main의 [SG2/backport 안내](docs/LEGACY_MAIN_README.md), `files/`, `original/`,
  `loop/` 참고 자료는 보존했습니다. 현재 SH5 설치는 `hardware/` 경로를 사용하며
  오래된 `loop/ffw_loop_streamer`로 로봇의 설치 패키지를 덮어쓰지 않습니다.
