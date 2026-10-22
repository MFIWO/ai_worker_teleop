# SH5 팔 + 손 배포 및 실행 (2026-10-22)

현재 입력: Quest 맨손 손목 → 기존 wrist-delta 팔 계산, 손가락 → ROBOTIS native hand retarget.
실제 로봇은 `sh5 --arm --hands` / `run_controller.sh sh5 --hands`를 사용한다. SG2 전용 `--start`는 사용하지 않는다.

## 이번 설정

| 항목 | sim | 실제 로봇 |
|---|---|---|
| 손목 목표 이동 속도 | 0.2 → 2.0 m/s | 0.3 → 3.0 m/s |
| 손목 목표 회전 속도 | 1 → 10 rad/s | 1.5 → 15 rad/s |
| VR/관절/TF 최신 입력 기준 | 0.5 → 5초 | 0.5 → 5초 |
| 광학 손 입력 기준 | 0.35 → 3.5초 | 위 VR 기준 사용 |
| sim 브리지 입력/명령 기준 | 0.4 → 4초 | 해당 없음 |
| 목표 유지 후 최종 정지 유예 | 별도 유예 없음 | 5 → 50초 |
| 네이티브 관절 피드백 타임아웃 | 0.5 → 5초 | 기존 override 5 → 50초 |
| 손가락 추가 소프트웨어 속도 상한 | 3 → 30 rad/s, URDF 값과 작은 쪽 | 기존 native hand retarget 유지 |

관절 각도와 URDF 관절 정격 속도는 유지된다. 팔 관절을 정격 속도의 10배로 구동하는 변경이 아니다.
입력이 끊겼을 때 이전 목표가 더 오래 유지될 수 있다. 물리 드라이버의 내부 제한은 변경하지 않았다.
작업 공간 제한, 충돌/CBF, 비정상 입력, 입력 점프와 reference checker는 남아 있다. sim의 추가 위치 추종 오차 0.25m 및 자세 오차 90도 정지는 해제했다.
실제 로봇용 속도/지연 상수는 HardwareXRPublisher와 gate가 공유하므로 같은 launcher의 SG2 모드에도 적용된다. 기존 SG2 실험용 sim 클래스 기본값은 유지된다.

코드 변경만 완료했다. 이번 변경 후 동작 테스트, 실제 로봇 실행, Git commit/push는 수행하지 않았다.
기존 프로파일 대신 controller를 재시작해 새 프로파일을 생성하고 PC/로봇 양쪽 소스를 함께 갱신해야 한다.

## 1. 개발 PC에서 Git push

```bash
cd ~/Downloads/external_repos/ai_worker_teleop
git status --short
git add -u
git add hardware/SH5_DEPLOY.md \
  robotis_vuer/robotis_vuer/vr_publisher_sh5_sim.py \
  sim/__init__.py sim/build_sh5_controller.py sim/check_sh5_sim.py \
  sim/prepare_sh5_sim.py sim/run_sh5_hybrid_sim.sh sim/run_sh5_physx_offline.sh \
  sim/sh5_hand_joint_audit.py sim/sh5_hybrid_sim.launch.py \
  sim/sh5_native_hand_retarget_probe.py sim/sh5_sim_bridge.py \
  sim/sh5_sim_container.sh sim/test_sh5_hybrid_runtime.py
git diff --cached --stat
git commit -m "Relax teleop software limits and include SH5 simulation"
git push origin HEAD:robotis-applications
```

## 2. 미니 PC에서 clone 및 PC 컨테이너 설치

기존 `robotis-applications`, 로봇의 `ai_worker` 컨테이너와 ROS Jazzy/ROBOTIS 설치를 사용하는 절차다. 빈 OS에 전체 시스템을 설치하는 절차가 아니다.
로봇에는 ROS 패키지 `ffw_bringup`, `ffw_description`, `rmw_zenoh_cpp`, `cyclo_motion_controller_ros`, `cyclo_motion_controller_ros_py`, `dynamixel_interfaces`와 기존 Cyclo C++ 의존성/colcon/CMake가 있어야 한다.
빌드는 `/root/ros2_ws/src/cyclo_control/cyclo_motion_controller_ros`의 기존 소스를 사용한다. Python 의존성은 numpy/scipy/PyYAML/pinocchio/rclpy이다.
기존 SH5 follower와 native hand retarget을 실행했던 이미지에는 해당 환경을 재사용한다. clone한 저장소만으로 없는 의존성을 오프라인 설치할 수는 없다.

```bash
mkdir -p ~/Downloads/external_repos
cd ~/Downloads/external_repos
git clone --branch robotis-applications --single-branch \
  https://github.com/MFIWO/ai_worker_teleop.git ai_worker_teleop_deploy
cd ai_worker_teleop_deploy
bash hardware/install_pc.sh
```

이후 각 PC 터미널에서 아래 두 값을 설정한다. ROBOT_IP의 예시 값은 실제 로봇 주소로 변경한다.

```bash
PC_IP=192.168.6.2
ROBOT_IP=192.168.6.3
```

PC IP가 바뀌었거나 인증서가 맞지 않을 때:

```bash
docker exec -it robotis-applications bash -lc '
  set -e
  cd /root/ros2_ws
  bash /root/gen_cert.sh "$1"
' bash "$PC_IP"
```

## 3. 인터넷 없는 로봇으로 rsync, 설치, 최초 빌드

동작 중인 이전 teleop/controller는 먼저 종료한다. 아래 복사/빌드는 로봇을 시작하지 않는다.

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
rsync -av --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
  --exclude='cert.pem' --exclude='key.pem' \
  ./ "robotis@$ROBOT_IP:~/ai_worker_teleop_sh5/"
ssh "robotis@$ROBOT_IP" \
  'cd ~/ai_worker_teleop_sh5 && bash hardware/install_robot.sh'
ssh "robotis@$ROBOT_IP" 'docker exec -i ai_worker bash -s' <<'ROBOT_BUILD'
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
cd /workspace/ai_worker_xr
python3 -m hardware.build_sh5_hands
ROBOT_BUILD
```

빌드는 최초 1회 필요하다. 빌더가 기존 바이너리 검증에 성공하면 재사용한다. 기존 소스와 바이너리가 다르다는 오류가 있을 때만 마지막 명령에 `--rebuild`를 붙인다. 이번 Python/설정 변경만으로 C++ 재빌드가 필요한 것은 아니다.

## 4. 실제 실행: PC의 별도 터미널 4개

각 터미널에서 `ROBOT_IP`를 설정한다. 마지막 Vuer 터미널에는 `PC_IP`도 설정한다.

A — 로봇 Zenoh router. 이미 로봇 컨테이너에서 실행 중이면 중복 실행하지 않는다.

```bash
ROBOT_IP=192.168.6.3
ssh -t "robotis@$ROBOT_IP" 'docker exec -it ai_worker bash -lc "
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=30
  ros2 run rmw_zenoh_cpp rmw_zenohd
"'
```

B — SH5 follower. 기존 follower가 실행 중이면 중복 실행하지 않는다.

```bash
ROBOT_IP=192.168.6.3
ssh -t "robotis@$ROBOT_IP" 'docker exec -it ai_worker bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /root/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ros2 launch ffw_bringup ffw_sh5_follower_ai.launch.py
"'
```

C — 로봇 팔 컨트롤러 + native SH5 hand retarget. B의 초기화 완료 후 실행한다.

```bash
ROBOT_IP=192.168.6.3
ssh -t "robotis@$ROBOT_IP" 'docker exec -it ai_worker bash \
  /workspace/ai_worker_xr/hardware/run_controller.sh sh5 --hands'
```

D — PC Vuer: 맨손 손목으로 팔, 손가락으로 SH5 손.

```bash
PC_IP=192.168.6.2
ROBOT_IP=192.168.6.3
docker exec -it robotis-applications bash \
  /root/ros2_ws/src/robotis_applications/run_xr_hardware_pc.sh \
  sh5 --arm --hands --server-ip "$PC_IP" --robot-ip "$ROBOT_IP"
```

Quest에서 `https://PC_IP:8012?ws=wss://PC_IP:8012`에 접속한다. 예: `https://192.168.6.2:8012?ws=wss://192.168.6.2:8012`.
양손이 보이게 한 상태에서 한 손 pinch + 반대 손 squeeze를 3초 유지하면 시작/정지한다. 반복할 때는 제스처를 한 번 풀어준다. 별도 hand retarget 실행은 필요 없다.

## 5. sim 실행

기존 hybrid 폴더에도 이번 설정을 반영했다. 위 Git 저장소에도 같은 sim 진입점을 포함했으므로 새 clone에서는 아래 경로로 실행한다.
Isaac Sim 5.1, IsaacLab, cyclo_lab 및 기존 DDS/Python 환경은 별도 설치되어 있어야 한다. 경로는 `ISAAC_SIM_ROOT`, `ISAAC_LAB_ROOT`, `CYCLO_LAB_ROOT`로 변경할 수 있다.

A:

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash sim/run_sh5_physx_offline.sh --gui
```

B — A가 준비되면:

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
bash sim/run_sh5_hybrid_sim.sh setup
bash sim/run_sh5_hybrid_sim.sh controller
```

C:

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
PC_IP=192.168.6.2
bash sim/run_sh5_hybrid_sim.sh vuer --server-ip "$PC_IP"
```

기존 실행 중인 sim controller/Vuer에는 파일 변경이 자동 반영되지 않는다. 두 프로세스를 종료 후 재실행해야 한다. PhysX는 그대로 둘 수 있다. sim은 FastDDS/domain232, 실제 로봇은 Zenoh/domain30이다. Vuer의 동일 포트 8012를 쓰므로 sim과 실제 Vuer를 동시에 실행하지 않는다.

## SH5 어깨 5mm 정지 조건 수정

첨부 sim 로그에서 READY 약 20초 후 `shoulder/base moved during session`으로 중단된 것을 확인했다.
SH5 sim과 실제 `--hands` publisher는 시작 시 저장한 어깨 좌표와 현재 좌표가 5mm 이상 다르다는 이유만으로 정지하지 않는다.
첫 초과 시 좌우별 실제 변위를 `[SH5 SHOULDER DRIFT]` 경고로 표시한다. 시작/재개 경로의 동일 조건에도 같은 정책을 적용한다.
원래 wrist-delta 기준점과 작업 공간은 유지하며 자동으로 다시 맞추지 않는다. 로그만으로 실제 어깨 좌표 변화 원인은 확정하지 않았다.
SG2의 해당 정지 조건 및 물리 관절/충돌/하드웨어 상태 검사는 그대로다.
현재 sim에서는 Vuer만 재시작하면 이 변경이 반영된다. 실제 로봇 경로는 PC 소스 갱신 후 Vuer 재시작이 필요하다.


## SH5에 남아 있던 SG2 실험 제한 제거

`near-extension joint4 command exceeds experiment bound`는 SH5 sim이 상속한 SG2 실험용 callback에서 발생했다.
기준은 joint4 > -0.45 + 0.005 rad였다. SH5 sim은 이제 XRResumePublisher의 일반 명령 수신 처리를 직접 사용한다.
관절 각도 검사는 원래대로 네이티브 QP 및 SimulationBridge의 실제 URDF 범위로 유지한다.
실제 로봇 HardwareXRPublisher는 이미 동일 SG2 실험 callback을 우회하고 물리 프로파일 각도를 검사하고 있었다.
sim의 추가 0.25m/90도 추종 오차 정지도 실제 HardwareXRPublisher와 같이 해제했다.
0.5m clutch 정지는 현재 SH5 ACTIVE 실행 경로에서 사용하지 않는다.

아직 정지/시작 거절이 가능한 조건: SH5 입력 점프(0.30m 또는 1.2rad),
native reference checker의 목표 점프(현재 설치 config 0.1m/30도), 입력/피드백 지연(위 변경값),
비정상/누락 입력, 중복 publisher, 모델/TF 불일치, startup 정렬/정지 확인 및 startup timeout,
URDF 범위 이탈, 충돌/CBF 또는 하드웨어 오류. workspace shell은 목표를 제한한다. SH5의 기존 0.94 비율은 아래 수정으로 해제했다.
이번 수정은 이 조건 전체를 해제하는 변경이 아니다. Vuer 재시작으로 sim에 적용하며 C++ 재빌드는 필요 없다.


## joint6 경계 수치 오차 및 SH5 도달 거리 수정

실행 프로파일의 arm_l_joint6 상한은 1.57 rad, 로그의 명령은 1.57002655510672 rad였다.
초과량 0.0000265551 rad(약 0.00152도)가 기존 1e-5 rad 허용값을 넘었다.
SH5 sim 및 실제 gate/PC 각도 검사의 허용 오차를 1e-4 rad로 맞췄다.
이 범위 내 초과는 실제 URDF 경계로 보정하여 전달하며, 더 큰 초과 및 비정상 입력은 계속 거부한다.
실제 URDF 관절 각도 범위는 넓히지 않았다. SG2 hardware의 기본 수치 허용 오차도 유지한다.

SH5의 손목 목표 공간은 기본 94% 외곽 반경 대신 측정된 두 구간 길이 합에서 1e-5m를 뺀 반경까지 허용한다.
손목 기준점, heading, 입력 scale은 바꾸지 않는다. 완전히 직선인 초기 자세의 수학적 특이점 검사는 유지한다.
이 공간은 단순한 기하 근사이므로 손목 자세/실제 관절 범위/충돌에 따라 QP가 팔을 완전히 펴지 못할 수 있다.
실행 프로파일의 팔꿈치 joint4 범위는 [-2.9361, 1.0786]rad로, 이전 -.7/-.45 실험 상한은 사용하지 않는다.

적용: sim은 controller와 Vuer 둘 다 재시작(PhysX 유지 가능, setup 재빌드 불필요).
실제 로봇은 PC install_pc.sh, 로봇 rsync/install_robot.sh 후 controller/Vuer를 모두 재시작한다.
새 workspace_fraction=1.0 프로파일이 필요하므로 기존 controller를 그대로 둔 채 PC만 갱신하면 안 된다.
이번 변경 후 동작 테스트 및 로봇 실행은 하지 않았다.


## 불완전한 HAND_MOVE 처리

`hand input must contain 25 finite 4x4 poses`는 HAND_MOVE 데이터가 파싱되지 않아 즉시 정지하던 경로였다.
SH5 sim 및 실제 손 통합 publisher는 양손 입력을 먼저 검증하고, 불완전한 프레임은 경고 후 버린다.
이 경우 정상 입력의 수신 시각을 갱신하거나 지우지 않으며 새 팔/손 명령도 발행하지 않는다.
정상 프레임이 기존 타임아웃 내 돌아오면 기존 세션에서 처리한다. 입력 점프 등 별도 검사는 유지한다.
누락이 계속되면 sim 광학 입력 3.5초, 실제 경로의 기존 5초/50초 유지·정지 정책이 적용된다.
오류 프레임으로 제스처 유지 시간이 누적되지 않도록 제스처 타이머와 손 필터는 초기화한다.
컨트롤러/브리지 오류와 retarget 계산 오류까지 무시하는 변경은 아니다.
sim은 Vuer만 재시작하면 된다. 실제 경로는 install_pc.sh로 PC 컨테이너 갱신 후 Vuer 재시작이 필요하다.
이번 수정은 문법/정적 검토만 수행했고 동작 테스트는 하지 않았다.


## 최종 SH5 sim / 실제 팔·손 경로 대조

2026-10-22: 손목 위치 점프 기준을 15cm에서 30cm로 변경했다. 회전 점프 기준 1.2rad는 유지한다.
점프 정지 로그에 좌우 손 구분, 실제 변위/회전 및 각각의 기준을 표시한다.
이 설정은 SH5SimulationPublisher와 실제 `sh5 --arm --hands`의 SH5HandsHardwarePublisher에 적용된다.
기존 SG2 모드의 위치 점프 기준은 15cm로 유지한다.

| 수정 항목 | SH5 sim | 실제 SH5 팔+손 |
|---|---|---|
| 위치 점프 | 30cm | 30cm |
| 어깨 5mm 이동 정지 | 경고만 | 경고만 |
| SG2 joint4 -.45 실험 상한 | 해당 callback 우회 | 기존부터 물리 프로파일 검사 사용 |
| 손목 추종 오차 25cm/90도 정지 | 해제 | 해제 |
| 손목 도달 거리 94% | 해제, 기하 길이 합-10um | 동일, 프로파일 fraction=1.0 |
| 팔 각도 경계 수치 오차 | 1e-4rad 이내 정확한 경계로 보정 | PC 검사와 로봇 gate 동일 |
| 불완전한 HAND_MOVE | 프레임 폐기, 기존 타임아웃 유지 | 동일 |
| 추가 속도/지연 기준 | 앞 표의 10배 값 | 앞 표의 10배 값 |

실제 경로에는 sim 브리지의 추가 손가락 3rad/s 제한이 원래 없으며, native 손 retarget/드라이버를 그대로 사용한다.
네이티브 reference checker의 목표 점프 검사와 입력 회전 점프, URDF 각도/정격 속도, 충돌/CBF 등은 유지한다.
PC install_pc.sh는 바뀐 모든 publisher/수학 모듈과 xr_contract를 복사하고, 로봇 install_robot.sh는 make_profile/command_gate/xr_contract를 복사한다.
이 대조는 소스 및 배포 경로 확인이며 실제 로봇 구동 검증은 아니다.

최신 코드를 받은 뒤 반드시 PC 설치와 로봇 rsync/설치를 모두 수행하고 controller/Vuer를 재시작한다.
이번 Python 변경으로 SH5 C++ 바이너리를 다시 빌드할 필요는 없지만, 최초 hands 빌드가 없는 로봇은 앞의 최초 빌드 절차가 필요하다.
