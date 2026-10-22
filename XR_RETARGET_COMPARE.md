# XR / SG2 팔 Retargeting 비교와 선택형 개선 모드

작성일: 2026-10-13. 기존 SG2/SH5 publisher, neck retarget, Cyclo C++/Python
제어기, URDF/SRDF, 관절 제한은 덮어쓰지 않았다. 새 모드는 시뮬레이션 전용이다.

## 1. 실제 비교한 소스

| 구분 | 고정 revision / 파일 |
|---|---|
| `MFIWO/xr_tele` main | `9faf1aafb81765896973527c6488f088252a2b50` |
| H1_2 / H2 IK | `teleop/robot_control/robot_arm_ik.py`, `H1_2_ArmIK`, `H2_ArmIK` |
| XR AI Worker 분기 | `agent/ai-worker-hx5-teleop`, `28d35e311581b30540929d262f5727791d17a6c6` |
| XR AI Worker 변환/IK | `teleop/robot_control/robotis_ai_worker.py`, `AIWorkerArmIK._target`, `AIWorkerVirtualLeaderIK` |
| 현재 PC Vuer | `robotis_vuer/robotis_vuer/vr_publisher_sg2.py`, `vr_publisher_sg2_neck.py` |
| 실제 설치 Cyclo | `ai_worker` 컨테이너 `/root/ros2_ws/src/cyclo_control` 및 해당 install symlink |
| 비교 기준 PC branch | `robotis-applications`, 작업 시작 HEAD `46257ca` |

참고 원문:

- https://github.com/MFIWO/xr_tele/blob/9faf1aafb81765896973527c6488f088252a2b50/teleop/robot_control/robot_arm_ik.py
- https://github.com/MFIWO/xr_tele/blob/28d35e311581b30540929d262f5727791d17a6c6/teleop/robot_control/robotis_ai_worker.py
- https://github.com/MFIWO/ai_worker_teleop/tree/robotis-applications

**재현성 한계:** H1_2/H2가 사용자 실환경에서 잘 작동한다는 관찰과,
이번 소스 비교는 구분한다. 이번 작업에서는 Unitree 실기를 실행하지 않았다.
main이 호출하는 TeleVuerWrapper 인자 일부와 gitlink
`b6ed5db2f08eabd0c49135fd565bc7f5a0c93573`의 생성자가 일치하지 않았다.
따라서 해당 GitHub checkout 전체가 사용자 성공 환경과 동일하다고 단정하지 않는다.

## 2. H1_2 / H2가 현재 SG2와 다른 핵심

H1_2 목적함수(소스 703–741):

```text
J = 50 * ||wrist_position_error||²
    + ||log(R_fk R_target^T)||²
    + 0.02 * ||q||²
    + 0.1 * ||q - q_last||²
```

H2는 회전 항 계수가 `0.8`이고, 나머지 구조는 유사하다. 두 구현은
매 solve에서 실측 관절값을 초기 추정과 연속성 기준에 사용하며,
사람의 어깨/팔꿈치 위치를 필수 목표로 하지 않는다.

| 항목 | H1_2 / H2 | 기존 SG2 경로 |
|---|---|---|
| 주요 입력 | 양손목 6D pose | 사람 어깨·팔꿈치·손목 6개 pose |
| 손목 목표 생성 | 손목 pose 중심 | 사람 팔 두 구간의 정규화 방향을 로봇 길이로 재구성 |
| 사람 팔꿈치 오류 | 필수 입력 아님 | 로봇 손목 목표 자체에 전파됨 |
| 초기화 | 실측 q 기반 반복 IK | 팔 목표를 절대 변환 후 startup 오차 검사 |
| 연속성 | q regularization / q-last / 필터 | Cyclo 속도 QP + 기존 목표/관절 상태 유지 |
| 팔꿈치 제약 | 기본 IK 목적에 별도 사람 팔꿈치 항 없음 | 설치 config elbow position weight=8 |

H1_2의 손목 EE `+0.05m` local-X, 팔 길이 scale `1.25`, H2의 scale 호출
비활성화, 각 로봇 관절 제한은 **SG2에 복사하지 않았다**. 또한 H1_2/H2의
가중 이동평균은 동일한 최신 입력을 skip하여 정지 후 필터가 완전히 수렴하지
않을 수 있어 그대로 가져오지 않았다. 새 모드에는 시간 기반 저역통과와
회전군 SO(3) 보간을 사용한다.

## 3. 현재 SG2의 문제를 일으킬 수 있는 소스상 원인

### 3.1 머리 방향이 팔 방향에 들어간다

기존 `vr_publisher_sg2.py`는 각 world pose에 매 프레임의
`head_inverse_matrix`를 곱한다. 팔은 가만히 두고 머리만 기울이거나
돌려도 head-relative 팔 방향은 바뀐다. neck 수정은 pitch/roll을 제거하지만
`gravity_arm_frame.py`가 head yaw를 매 프레임 갱신하므로 yaw 영향은 남는다.

### 3.2 단순 공통 높이 오프셋만의 문제는 아니다

기존 direction retarget은 다음 식을 쓴다:

```text
u = normalize(e_h - s_h)
f = normalize(w_h - e_h)
e_goal = shoulder_robot + L_upper * u
w_goal = e_goal + L_forearm * f
```

모든 점에 동일한 translation을 더하면 차분에서 소거된다. 따라서
head translation, 공통 camera/base shift, 공통 lift offset만을 이 경로의
손목 높이 문제로 설명하는 것은 부정확하다. 하지만 full head 회전,
shoulder=scapula 대신 arm-upper를 사용해야 하는 차이, **손목에만 추가된
Z=-0.10m**, controller-local `0.12m` 위치 보정은 소거되지 않는다.
직접 `/wrist_pose -> /goal_pose` remap 경로에서는 절대 높이 오프셋도 그대로 남는다.

### 3.3 손목 목표가 추정 팔꿈치와 양손 거리 보정에 의존한다

사람 elbow가 틀리면 위 식의 손목 목표도 틀린다. QP에서 elbow weight만
줄여도 이미 손목 목표에 들어간 오류는 제거되지 않는다. 설치된 기존
retargeter의 wrist-span 보정은 양손 목표를 함께 수정한다. 한 손만
움직였는데 다른 손 목표까지 바뀔 가능성이 있는 구조다.

`_enforce_human_wrist_relative_orientation`은 현재 입력/출력 관계에서는
대수적으로 원래 좌우 방향을 돌려주는 항등식이다. 이번 분석에서는 이를
양손 회전 오류의 원인으로 잘못 분류하지 않았다.

### 3.4 실제 피드백과 제어기 내부 목표 상태를 구분해야 한다

설치 C++의 `/[lr]_gripper_pose`는 **inactive일 때 실측 FK**이지만,
active일 때는 `q_desired`의 FK다. 새 캘리브레이션은 inactive 이후 생성된
timestamp를 가진 pose만 허용하고, 실측 `/joint_states`와 PhysX TF를 함께
검사한다. 이전 로컬 C++보다 실제 Docker C++가 더 최신이며, 설치본에는
inactive / reactivate 시 `syncCommandStateToFeedback()`가 있다.

실제 Docker URDF SHA256:
`fdf2734725cbb5b7c2ebbf36d314b6e00dfc902ba8be331700df48284a84e063`.
옛 `retarget_validation/review_cyclo_651aa55.urdf`와 joint7 좌우 제한이
다르므로 그 옛 파일의 제한을 새 런타임에 덮어쓰지 않았다.

## 4. 구현한 개선 모드: `XR Delta`

XR AI Worker의 `_target`에서 **측정 시작 자세를 기준으로 한 이동/회전량
변환 원리**를 적용했다. H1_2/H2 IK 전체나 새로운 관절 solver를 이식한 것은 아니다.
기존 Cyclo QP, 충돌 검사, 관절 제한, reference checker를 그대로 사용한다.

X+A에서 캡처한 고정 수평 좌표변환을 `F`, 컨트롤러 기준 pose를
`p_c0, R_c0`, 실측 로봇 손목 기준 pose를 `p_r0, R_r0`라 하면:

```text
p_goal = p_r0 + scale * F * (p_c - p_c0)
R_goal = F * R_c * R_c0^T * F^T * R_r0
```

연결 순서:

```text
Quest controller pose
 -> 시작 때 고정한 수평 좌표계의 변화량
 -> 시작 실측 SG2 link7 pose에 적용
 -> 위치/회전 저역통과 + 변화속도 제한
 -> /[lr]_goal_pose 및 연속 로봇 elbow hint
 -> 기존 Cyclo QP
 -> 기존 SG2 DDS Bridge / PhysX
```

특징:

- 캡처 시 첫 손목 목표가 로봇의 정지 자세와 일치한다.
- 시작 후 머리 회전·이동이 고정된 컨트롤러의 목표를 바꾸지 않는다.
- 사람의 추정 shoulder/elbow는 사용하지 않는다. heading 캡처용 head와
  양 컨트롤러의 추적 신선도는 계속 확인한다.
- 양 손목은 독립적이며, 양손 거리 강제 보정을 하지 않는다.
- controller-local 12cm와 고정 90도 손목 보정을 사용하지 않는다.
- 팔꿈치는 캡처한 로봇의 실제 S/E/W와 연속적인 굽힘 평면으로 생성한
  기하학적 보조 목표를 쓴다. **완전한 SG2 7-DoF 해가 아니라 힌트**다.
- 기본 목표 속도 0.20m/s, 회전 속도 1rad/s, 추적/관절 피드백 timeout0.5s.
  입력 jump, 잘못된 회전, 오래된 TF, 중복 목표 발행기, 과도한 추종 오차에서 정지한다.
- Y+B로 정지 후에만 재캡처한다. 활성 중 X+A는 무시한다.
- 손가락/그리퍼/trigger/베이스/lift/head 명령은 발행하지 않는다.
- SIGINT/SIGTERM 종료 시 ROS context를 닫기 전에 `/reactivate=False`를 보낸다.

**달라지는 동작 의미:** 사람의 절대 팔 자세를 완전히 복제하는 모드가 아니라,
시작한 로봇 자세에서 손목의 이동/회전량을 따라가는 Cartesian teleop이다.
사람이 팔을 어디에 두었는지보다, 캡처 이후 얼마나 어떻게 움직였는지가 중요하다.

## 5. 실행: 바뀌는 두 군데만

기존 SG2 Isaac Sim 명령과 DDS 설정은 유지한다. 실제 사용자 테스트는 domain231.
기존 Vuer와 AI Worker VR launch를 먼저 Ctrl+C로 종료하여 중복 실행하지 않는다.

### AI Worker Docker — `arm:=false` 한 인자 추가

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
  controller_type:=vr hand:=false arm:=false
```

`hand:=false`는 손가락 retarget만 끈다. 추가한 `arm:=false`는 기존 Python
팔 retargeter를 끄고 새 PC 모드의 robot-frame 목표가 중복 변환되지 않게 한다.
이미 설치된 launch가 이 옵션을 지원하므로 재빌드/소스 교체가 필요 없다.

### ROBOTIS Applications Docker — 새 선택형 스크립트

```bash
export ROS_DOMAIN_ID=231
bash /root/ros2_ws/src/robotis_applications/run_sg2_xr_sim.sh
```

속도를 낮춘 시험:

```bash
bash /root/ros2_ws/src/robotis_applications/run_sg2_xr_sim.sh -p xr_goal_speed:=0.10
```

Quest에서 컨트롤러 tracking을 잡은 뒤 편한 팔 자세로 X/A에서 손을 떼었다가
X+A를 누른다. `[XR CALIBRATED]`, AI Worker의 `Startup check passed`,
`Controller activated`를 확인한다. 기존 3초 대기 및 8초 slow-start ramp는
유지한다. Y+B로 멈추고 재기준이 필요하면 다시 X+A를 누른다.

시뮬레이터 reset, 모드 전환, 큰 추종 오차 후에는 AI Worker launch도 재시작한다.
reference checker가 이전 goal을 기억하므로 0.1m/30도 이상의 기준 변화는
첫 재캡처를 거부할 수 있다. 안전 검사를 무시하거나 자동 해제하지 않는다.

롤백: 새 Vuer를 종료하고 AI Worker 명령에서 `arm:=false`를 제거한 뒤,
기존 `ros2 launch robotis_vuer vr.launch.py model:=sg2`로 실행하면 된다.

## 6. 검증 범위

- 순수 수학/좌표/회전/속도/기하학적 경계 unit test15개.
- 실제 rclpy node로 inactive feedback, delayed source timestamp,
  active X+A 재캡처 차단 등을 검사하는 guard test5개.
- **실제 Docker의 Cyclo C++ QP와 실제 Isaac Sim PhysX**를 domain232에서
  연결하고, 합성 WebXR 이벤트를 실제 publisher callback에 입력했다.
- 머리만 움직이기, 왼손목 5cm 전진, 손목15도 회전, Y+B, 정상 재캡처,
  활성 중 X+A, NaN 입력 정지를 검증했다.
- 실제 Quest 브라우저 착용으로 같은 동작을 수행한 실측은 아직 아니다.
- 새 Vuer server의 LAN HTTPS 및 WSS handshake를 확인했지만 이것은
  Quest 센서 스트림/VR 권한/사용자 기기 인증서 신뢰 검증과 다르다.

실험 원본:
`docker/workspace/xr_delta_release.json` (합성 입력 표기, 원시 시계열 포함).
로그: `/home/kimm/Downloads/.retarget-review/results/release/`.

최종 실행은 `passed=true`, 9개 통합 판정 모두 통과했다.

| 시험 | 실제 결과 |
|---|---:|
| 머리만 회전/이동, 손 고정 | 왼손목 목표 이동 0m |
| 왼손 컨트롤러 50mm 전진 | 안정화 기준 실제 손목 49.161mm 전진 |
| 전진 종료 손목 목표–PhysX 오차 | 2.795mm |
| 같은 구간 오른손목 실제 drift | 0.011mm |
| 위치 고정, 손목15도 회전 | 목표 위치 drift 약5.6e-17m |
| 회전 종료 목표–PhysX 방향 오차 | 0.283도 |
| 활성 중 X+A, Y+B 정지, NaN 입력 정지 | 통과 |

안정화 기준은 기존 3초 지연과 8초 slow-start가 끝난 후, 손을 움직이기
직전이다. 원본 JSON에는 slow-start 도중(t=5초)을 기준으로 한 결과도
그대로 남아 있다(그 기준의 오른손목 총 drift는7.36mm). 유리한 값으로
원시 시계열을 대체하거나 warm-up 이동을 숨기지 않았다. 수치는 합성
단일 시험의 측정이며 모든 사용자·속도·작업공간에 대한 정확도 보장은 아니다.

### 남는 한계

SG2 link4→link7 유효 거리는 wrist joint6 각도에 따라 변한다. 이번 기하학적
elbow hint의 캡처 길이가 모든 손목 방향에서 정확하다고 주장하지 않는다.
큰 손목 굽힘·최대 도달 거리·자가충돌 자세는 별도 검증이 필요하며,
실제 QP가 목표를 완전히 달성하지 못하면 새 모드도 정지할 수 있다.
H1_2/H2의 solver로 교체하거나 IK weight/관절 제한을 완화한 작업이 아니다.

## 7. 네트워크/인증서 확인

PC 공유기 측 IP는 이번 점검에서 `192.168.6.2`, Vuer port는8012였다.
기존 certificate SAN에는 institute IP `10.116.1.37`만 들어 있었다.
같은 mkcert CA를 유지하고 기존 pair를 컨테이너 내
`/root/.local/share/vuer-cert-backups/20261013_104049/`에 백업한 뒤,
`192.168.6.2`, `10.116.1.37`, localhost,127.0.0.1을 포함한 server certificate로 갱신했다.

```text
https://vuer.ai?ws=wss://192.168.6.2:8012
```

컨테이너에서 같은 CA로 IP까지 검증한 HTTPS HTTP200과 WSS handshake가
성공했다. Quest가 아직 이 CA를 신뢰하지 않으면 기기 측 신뢰 설정은 필요하다.
certificate/key는 `.gitignore`로 제외되어 있으며 Git에 올리지 않는다.

## 8. 새 파일과 보존 범위

```text
run_sg2_xr_sim.sh
XR_RETARGET_COMPARE.md
robotis_vuer/robotis_vuer/xr_arm_targets.py
robotis_vuer/robotis_vuer/vr_publisher_sg2_xr.py
robotis_vuer/test/test_xr_arm_targets.py
robotis_vuer/test/test_xr_publisher_safety.py
robotis_vuer/test/xr_sim_smoke.py
```

기존 uncommitted neck 코드와 `ai_worker_teleop` main 작업은 그대로 두었다.
새 파일은 로컬 추가 상태이며 이번 작업에서 git commit/push는 하지 않았다.
참고한 XR 소스의 Apache-2.0 attribution과 revision은 새 수학 모듈에도 표기했다.
