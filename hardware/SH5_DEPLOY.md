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
작업 공간 제한, 충돌/CBF, 비정상 입력, 입력 점프, reference checker, sim의 위치 추종 오차 0.25m 및 자세 오차 90도 정지는 남아 있다.
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
