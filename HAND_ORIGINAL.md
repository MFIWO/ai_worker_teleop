# 손 추적의 수정 전 팔 처리 경로 비교

기존 PC `robotis-applications` 컨테이너를 사용한다. 현재 컨테이너는
`/home/kimm/Downloads/external_repos/robotis_applications`를 bind mount한다.
새 `robotis_app_2`용 Docker를 시작하거나 컨테이너를 재생성할 필요가 없다.

2026-09-18 확인: `robotis_applications`, `robotis_app_2`,
`ai_worker_teleop-export.gEpbRh`의 `vr_publisher_sh5.py`는 바이트 단위로 동일하다.
원래 SH5 손 추적 입력 코드는 변경하지 않았다.

이 비교 실행은 백포트한 `arm_retargeting_teleop`를 실행하지 않고,
수정 전 로봇 launch 백업을 그대로 실행한다. PC의 원본 SH5 손목 출력을
`/[lr]_goal_pose`로 연결하며, 팔꿈치는 원래 `/[lr]_elbow_pose`를 사용한다.
추가한 로봇 팔 길이 정규화 및 양손 간격 변환을 통과하지 않는다.
그리퍼 명령 전달 수정, 관절 제한, startup 검사 기준은 그대로다.
팔 처리 경로의 비교이며 전체 실행 파일을 과거 버전으로 되돌리는 작업은 아니다.

## 실행

기존 PC VR 서버와 로봇 VR 제어기 launch를 종료한다. follower bringup은 유지한다.
두 경로를 동시에 실행하지 않는다.

PC `robotis-applications` 컨테이너:

```bash
bash /root/ros2_ws/src/robotis_applications/hand_original_pc.sh
```

PC 스크립트는 먼저 양쪽 팔을 `xr_tele`의 SG2 ready 자세로 약 5초에 걸쳐 이동한다.
목표는 각 팔 `[0, 0, 0, -1.57, 0, 0, 0]` rad로, 팔꿈치만 약 90도 굽힌 자세다.
원본은 `xr_tele/teleop/robot_control/robotis_ai_worker.py`의
`AI_WORKER_SG2_READY_Q`이며, 같은 quintic 보간을 사용한다. 현재 측정 관절값에서
시작하고, 양쪽 follower의 `FollowJointTrajectory` action으로 팔 7관절만 보낸다.
그리퍼/목/리프트는 준비 동작에 포함하지 않는다.

`READY: both arms reached the pose` 출력 후에만 다음 로봇 VR 제어기를 시작한다.
기존 VR 제어기는 내부 목표 자세를 보관하므로 준비 동작 후 새로 시작해야 한다.
기존 VR/leader 제어기가 발견되거나 준비 동작/실제 관절값 확인에 실패하면
PC 스크립트는 hand 서버를 시작하지 않고 종료한다.

로봇 `ai_worker` 컨테이너 (`READY` 이후):

```bash
bash /workspace/quest_sg2_teleop/hand_original_robot.sh
```

Quest 페이지를 새로고침하고 맨손 추적으로 전환한다. 한 손 pinch와 다른 손
주먹을 3초 유지하면 활성화/정지가 전환된다. `hand:=false`여서 로봇 손가락
retargeting은 실행하지 않는다. SH5 기본 설정으로 목/베이스/리프트 명령도 꺼져 있다.
원래 좌표의 startup 오차가 기준을 넘으면 활성화되지 않을 수 있으며,
이 경우 위치/방향 오차 로그를 확인한다. 검사 기준을 완화하지 않았다.

## Controller 경로로 복귀

위 두 프로세스를 종료한 뒤 기존 명령을 사용한다.

PC: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 ros2 launch robotis_vuer vr.launch.py model:=sg2`

로봇: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr hand:=false`

일반 launch의 controller 팔 변환, X+A 유지, 트리거 그리퍼 수정은 유지된다.

## 준비 동작 검증

`prepare_sg2_ready_pose.py`를 PC 스크립트가 직접 실행하므로 ROS 패키지를 다시 빌드할
필요가 없다. `control_msgs` 의존성을 패키지 manifest에 추가했고 현재 PC 컨테이너에도
설치했다. `--dry-run`은 목표만 출력하며 ROS 노드나 명령을 생성하지 않는다.

ROS domain 231의 모의 follower로 양팔 action 전송, 실측 도달 확인 실패,
기존 VR 노드와의 충돌 차단을 검증했다. 궤적 끝점/시간/속도 제한도 테스트했다.
실제 로봇에서 이 준비 동작을 실행한 검증은 아직 하지 않았다.
준비 자세는 초기 위치 차이를 줄이려는 것이며, 손 추적 좌표나 방향 오차까지
보정하는 기능은 아니다. 원본 hand retargeting과 startup 검사 기준은 유지한다.

## PC에서 수동 로그 수집

기존 PC hand 서버, 로봇 follower 및 VR 제어기를 유지하고, PC 컨테이너의 별도 터미널에서 실행한다.
로깅하려고 `hand_original_pc.sh`를 다시 실행하면 준비 동작 단계가 실행되므로,
이미 구동 중일 때는 아래 로거만 추가한다.

```bash
bash /root/ros2_ws/src/robotis_applications/log_vr_pc.sh --seconds 60
```

60초 후 자동 저장되며 Ctrl+C로 종료해도 저장한다. 로그 파일은
컨테이너 `/workspace/teleop_logs/vr_arm_<UTC시간>.json`,
호스트 `/home/kimm/Downloads/external_repos/robotis_applications/docker/workspace/teleop_logs/`에 남는다.
`--output /workspace/teleop_logs/<새파일명>.json`으로 파일명을 지정할 수 있으며 기존 파일은 덮어쓰지 않는다.

로거는 ROS 명령을 발행하거나 teleop을 활성화하지 않는다. 원본 hand 서버는 활성화 중에만
어깨/팔꿈치/손목 목표를 발행하므로 비활성 상태에서는 해당 항목이 NO DATA 또는 STALE일 수 있다.
실제 목표 수집은 기존 teleop 활성 상태에서 진행하며 로봇은 기존 제어에 따라 움직인다.

실제 관절(리프트 포함), 양팔 명령, VR 어깨/팔꿈치/손목 목표, 제어기의 gripper pose와
수신된 reactivate 이벤트를 저장한다. 최신값 묶음을 약 10Hz로 기록하고 수신 시각/나이를 포함한다.
하드웨어 동기화나 모든 원본 메시지를 기록하는 rosbag은 아니다.
터미널은 1초마다 수신 상태와 VR 좌표를 표시하고, 로봇 어깨 FK 및 목표 오차 분석은 저장 파일로 수행한다.

## 통합 조작 터미널 (2026-09-19)

PC hand 실행 스크립트에 U 일시정지 래퍼를 연결했다. 원본 SH5 소스와 좌표 수식은
유지하며, 비활성 손에 0을 보내는 대신 마지막 목표를 유지한다.
실행 및 키 배치는 [OPERATOR_CONSOLE.md](OPERATOR_CONSOLE.md)를 따른다.
