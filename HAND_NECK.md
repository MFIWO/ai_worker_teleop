# 목 착용 hand teleop: 중력 수평 기준

PC 원본 vr_publisher_sh5.py는 변경하지 않았다. 새 NeckVRTrajectoryPublisher가
원본을 상속하며, 손목/팔꿈치/어깨의 publish_relative_pose 전에 동일한 회전 변환을
추가한다. hand_neck_pc.sh로 선택한다. controller+hand 혼합 모드는 추가하지 않았다.

WebXR world Y-up과 body-head +Y-forward를 사용한다. head 전방의 수평 투영으로
전방, 중력 반대 방향으로 위, 외적으로 왼쪽을 만든다. 원본의 head-relative
위치와 방향을 이 기준으로 함께 변환한 뒤 원본의 필터/scale/offset/우측 Z180도를
적용한다. head가 거의 수직이면 직전 수평 방위를 유지하고, 초기 방위가 없거나
head 회전행렬이 무효하면 팔 목표를 발행하지 않는다.

기기 기울기가 손 위치와 손목 방향에 직접 섞이는 문제를 수정했다.
고정 위치 오프셋, 체격 차이, body tracking 추정 오차 및 손/그리퍼 축 대응은
별도 문제다. 실제 로봇에서 모든 자세 오차가 사라졌다는 검증은 아직 하지 않았다.

## 실행 순서

기존 PC VR 서버와 로봇 VR 제어기를 종료한다. follower는 유지한다.
follower도 꺼져 있다면 로봇 ai_worker 컨테이너의 ROS 환경에서 먼저:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```

PC robotis-applications 컨테이너:

```bash
bash /root/ros2_ws/src/robotis_applications/hand_neck_pc.sh
```

양팔을 약 5초에 걸쳐 팔꿈치 -1.57rad 준비 자세로 이동한다.
READY 출력 후 로봇 ai_worker의 별도 터미널:

```bash
bash /workspace/quest_sg2_teleop/hand_original_robot.sh
```

로봇 측은 원본 팔 목표 처리 경로를 사용한다. PC만 목 착용 모드로 바뀐다.
Quest 페이지를 새로고침한다. 몸통 정면에 헤드셋을 고정하고 기존 pinch+반대손
주먹 3초 제스처로 활성화한다. 장치가 아래를 향하는 각도에 맞춰 팔을 일부러
내리지 말고, 바닥과 수평인 몸 앞 방향으로 팔을 움직여 비교한다.
관절 제한과 startup 위치/방향 검사 기준은 유지한다.

PC 별도 터미널에서:

```bash
bash /root/ros2_ws/src/robotis_applications/log_vr_pc.sh --seconds 60
```

새 진단 토픽 /vr/diagnostics/head_pose와 /vr/diagnostics/arm_reference_pose도
저장한다. 둘 다 frame_id=xr_world_y_up이며, head는 원본 body-head 축,
reference는 +X수평전방/+Y왼쪽/+Z중력상방을 XR world에 표현한 자세다.
진단은 유효 body event가 있으면 비활성 중에도 발행하지만 기존 팔 목표는 활성 중에만
발행한다. 로그는 /workspace/teleop_logs에 저장한다.

## 검증

- 변환/콜백 테스트 10개 통과: pitch/roll/yaw, 평행 이동, 무효/수직 head,
  좌우 및 어깨/팔꿈치/손목 전달, 원본 옵션 보존, 진단 자세.
- isolated ROS domain 231에서 실제 노드 생성과 두 진단 토픽 수신 확인.
  Vuer 서버 시작을 mock하고 teleop은 비활성 상태로 유지했다.
- 추가/수정 Python 파일 4개 ament_flake8 통과. 셸 문법 확인.
- 실제 로봇 이동, VR 서버 재시작, 자동 활성화는 수행하지 않았다.

원본 비교로 돌아가려면 PC에서 hand_original_pc.sh를 사용하고 동일한 순서를 따른다.

## 손을 내릴 때 추적 누락/급변 처리 (2026-09-18 추가)

목 착용 모드에만 적용한다. 원본 hand 및 controller 경로와 로봇 reference_checker
기준은 변경하지 않았다. 정상 입력은 수평 좌표 변환 후 기존 경로로 전달한다.

- 한쪽 손 또는 body 데이터가 0.25초 넘게 갱신되지 않거나 손목 행렬이 무효이면
  새 팔 목표를 보내지 않고 마지막 유효 목표를 유지한다. 추적 없는 손 위치를 추정해
  차렷까지 움직이지 않는다. 로봇은 마지막 목표로 수렴할 수 있으므로 즉시 정지 명령은 아니다.
- 손목/팔꿈치/어깨의 원본 변환 목표와 필터 후 목표를 모두 검사한다.
  이전 목표 대비 8cm/20도 초과 점프가 있으면 그 body 프레임의 양팔 목표 전체를 보류한다.
  기존 로봇 reference_checker의 10cm/30도 기준은 그대로다.
- 손 추적이 돌아온 후 마지막 허용 목표에서 5cm/15도 이내로 돌아오고,
  약 0.3초 동안 2cm/8도 이내로 안정되어야 다시 전달한다.
  멀리 떨어진 재인식 목표로 자동 보간하지 않으며, 기다리기만 해도 재개되지 않는다.
- 거부된 프레임은 필터 상태도 복원해 잘못된 입력을 향해 목표가 서서히 이동하지 않게 한다.
- PC 콘솔 및 /vr/diagnostics/arm_tracking_status:
  tracking=추종, tracking_missing=입력 누락,
  return_to_held_pose=손을 멈췄던 목표 근처로 돌려놓아야 함.
  log_vr_pc.sh도 이 상태를 저장/표시한다.

기존 코드로 이미 Reference divergence가 걸렸다면, 아래 수정이 그 fault를 자동 해제하지
않는다. PC VR 서버와 로봇 VR 제어기를 종료하고 follower만 유지한 채
hand_neck_pc.sh → READY → hand_original_robot.sh 순서로 다시 시작한다.
준비 스크립트는 실제 양팔을 90도 자세로 이동하므로 준비 단계를 인지하고 실행한다.

검증: 기존 수평 좌표 테스트 10개, tracking guard 테스트 7개,
격리 ROS domain 231에서 실제 hand/body callback 전체 경로 테스트 1개 통과.
사용자 로그의 16.3cm/13.3cm 급변은 보류되는 것을 모의 검증했다.
실제 시야 밖 손 추적 여부와 복구 동작은 사용자의 다음 teleop에서 검증해야 한다.

## 통합 조작 터미널 (2026-09-19)

PC hand 실행 스크립트에 U 일시정지 래퍼를 연결했다. 원본 SH5 소스와 좌표 수식은
유지하며, 비활성 손에 0을 보내는 대신 마지막 목표를 유지한다.
실행 및 키 배치는 [OPERATOR_CONSOLE.md](OPERATOR_CONSOLE.md)를 따른다.
