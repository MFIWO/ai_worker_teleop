# AI Worker VR → Loop 팔 관절 기록

2026-09-18 확인. 원본 리더용 ffw_sg2_rev1 설정은 유지하고 ffw_sg2_vr를 추가했다.
현재 로봇 SSH는 No route to host여서 로봇 설치 및 새 VR 실녹화는 미실행이다.

## action의 의미

- xr_tele/teleop/teleop_hand_and_arm.py: VR wrist → solve_ik → sol_q → arm_ctrl.ctrl_dual_arm.
  LoopRobotStreamer.send에 state=current_lr_arm_q, action=sol_q를 전달한다.
  episode JSON도 actions.left_arm/right_arm.qpos에 sol_q를 기록한다.
  별도 arm trace에는 requested/sent 명령도 기록하므로 sol_q가 actuator 측 최종 속도제한
  명령과 항상 같다고 보장하지는 않는다.
- 기존 master/leader: 리더가 발행하는 JointTrajectory를 Loop가 action으로 읽는다.
- 현재 AI Worker VR: 로봇 vr_controller가 IK/QP 계산 후 발행한 최종 JointTrajectory를
  같은 /leader/.../joint_trajectory 토픽에서 읽는다. 토픽 이름에 leader가 있어도
  실제 발행자는 VR 제어기다. 관절 각도 단위는 rad이며 /joint_states 실측값은 observation이다.
- 누락된 action에 observation을 복사하거나 0을 채우지 않는다.

## 분리된 VR layout

config/ffw_sg2_vr.yaml:
- ffw_sg2_vr.observation.left_arm.joint_position: arm_l_joint1..7 [7]
- ffw_sg2_vr.observation.right_arm.joint_position: arm_r_joint1..7 [7]
- ffw_sg2_vr.action.left_arm.joint_position: 최종 왼팔 명령 [7]
- ffw_sg2_vr.action.right_arm.joint_position: 최종 오른팔 명령 [7]
- left_gripper/right_gripper의 observation/action은 각각 별도 [1] 채널.
  맨손 VR에 그리퍼 명령이 없으면 해당 action만 None이며 팔 action은 유지된다.
- 이번 profile은 팔/그리퍼와 기존 카메라 3대에 집중한다. 머리/리프트/베이스 관절
  기록이 필요한 데이터셋은 추가 채널 설계를 해야 한다.
- source_key=robotis-vr를 사용해 기존 8차원 left/right 데이터와 구분한다.

## 포함 범위

`ffw_loop_streamer/`는 현재 로컬 패키지 소스와 VR profile 테스트다.
아래 installer는 로봇에 기존 `ffw_loop_streamer` 및 Loop SDK가 설치된 환경을 대상으로
VR YAML과 launch 인자만 추가한다. 최초 Loop 설치나 SDK 설치 도구가 아니다.
저장소 `main`의 루트에서 설치 명령을 실행한다. IP/호스트는 현장 환경에 맞춘다.

## 설치 및 실행

로봇 연결 복구 후 PC 호스트에서 1회:

```bash
bash loop/scripts/install_vr_profile.sh <robot-host>
```

VR 설정과 launch source_key 인자만 추가하고 해당 패키지를 빌드한다.
기존 launch 파일은 백업한다. 프로세스/서보/녹화를 자동 시작하지 않는다.

기존 follower + PC hand/controller VR + 로봇 VR 제어기를 구동한 다음,
로봇 ai_worker 컨테이너의 별도 터미널에서:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 \
ros2 launch ffw_loop_streamer loop_streamer.launch.py \
robot_type:=ffw_sg2_vr source_key:=robotis-vr \
loop_addr:=192.168.6.56:50051 fps:=30.0
```

Loop Recorder에서는 새 robotis-vr 소스와 기존 카메라 3대를 선택하고 30Hz로 기록한다.
새 channel layout이므로 기존 리더 데이터셋의 8차원 action과 합쳐 쓰지 않는다.
90도 ready 이동 및 VR 활성화를 끝내고 녹화를 시작한다. 준비 자세 action 명령은
/leader trajectory가 아닌 follower action server로 전달되므로 이 action 채널에 잡히지 않는다.
물리 리더를 같이 켜는 ffw_sg2_ai 통합 alias는 VR과 함께 실행하지 않는다.
기존 Loop streamer도 중복 실행하지 않는다.

현재 streamer는 최신 메시지를 캐시해서 30Hz로 기록한다. 추적 HOLD 또는 VR 정지 중에도
직전 명령값이 남을 수 있으므로 추종 중인 구간을 기록하고, tracking/activation 상태를
자동 에피소드 품질 검사에 쓰려면 별도 메타데이터 채널 추가가 필요하다.

## 검증

- VR profile 단위검증 4개 통과: 7관절 유지, 그리퍼 분리, observation→action 대체 방지,
  별도 채널 이름/토픽 연결.
- 저장된 2026-09-18 22:56:59 KST hand VR 로그를 mapper에 재입력:
  실제 명령이 신선한 96개 샘플 중 기존 설정 action 유효 0/96, 새 설정 96/96.
  이는 로컬 변환 검증이며 실제 Loop 전송/에피소드 저장 검증은 아니다.
- installer는 임시 디렉터리에서 원본 launch 구조 패치 및 재실행 검증 완료.

## 기존 카메라 저장 검증

2026-09-17 리더 방식으로 저장한 에피소드 3개에서 카메라 3대의 영상 총 9개를
전체 디코딩했다. 영상별 프레임 수와 각 에피소드의 robot parquet 행 수가 일치했다.
30fps, head 672x376, 양손목 240x424였으며 검은 프레임과 timestamp 역전은 없었다.
이는 기존 리더 녹화의 검증이다. 새 VR profile의 실제 저장 검증은 아직 필요하다.
원본 에피소드, 현장 이미지, NAS 경로 및 진단 로그는 저장소에 포함하지 않는다.
