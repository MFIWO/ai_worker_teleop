# AI Worker XR Retargeting

이 문서는 `xr_tele`의 wrist-centric teleoperation 아이디어를 참고해 만든
**AI Worker 전용 retargeting 방식**만 짧게 정리한다.

상세 비교와 실험 기록은 [XR_RETARGET_COMPARE.md](XR_RETARGET_COMPARE.md),
[SHOULDER_FREE.md](SHOULDER_FREE.md), [XR_FIXED_RESUME.md](XR_FIXED_RESUME.md)를 참고한다.

## 1. 핵심 아이디어

기존 AI Worker retargeting은 사람의 shoulder / elbow / wrist 위치를 이용해
로봇 팔의 목표 자세를 재구성했다. 이 방식은 사람 팔꿈치 추정 오차,
머리 기준 좌표계 변화, 사람과 로봇의 팔 길이 차이가 wrist 목표까지
전파될 수 있다.

AI Worker XR 방식은 `xr_tele`의 H1/H2 계열에서 확인한
**wrist pose 중심 제어**와 AI Worker 분기의 **초기 pose 기준 상대 변환**을
참고해 다음 구조로 바꿨다.

```text
Quest controller pose
        ↓
초기 calibration 기준 wrist translation / rotation delta
        ↓
AI Worker 실제 wrist pose에 delta 적용
        ↓
robot-frame left/right wrist goal
        ↓
기존 Cyclo QP
        ↓
AI Worker arm joints
```

즉, 사람의 절대 팔 자세를 그대로 복제하는 방식이 아니라
**캘리브레이션 이후 손목이 얼마나 움직였는지**를 로봇 손목에 대응시킨다.

## 2. 변환식

캘리브레이션 시 Quest controller pose를
`(p_c0, R_c0)`, 로봇 실제 wrist pose를 `(p_r0, R_r0)`,
Quest→robot 수평 좌표 변환을 `F`라고 하면 목표는 다음과 같다.

```text
p_goal = p_r0 + scale * F * (p_c - p_c0)

R_goal = F * R_c * R_c0^T * F^T * R_r0
```

따라서 X+A calibration 직후에는

```text
p_goal = p_r0
R_goal = R_r0
```

가 되어 첫 목표가 현재 로봇 자세에서 연속적으로 시작한다.

## 3. xr_tele에서 참고한 것 / 복사하지 않은 것

참고한 부분:

- wrist 6D pose를 arm retargeting의 주 목표로 사용하는 구조
- 매 순간 사람 elbow 위치를 필수 목표로 두지 않는 방식
- 시작 자세를 기준으로 상대 motion을 robot pose에 적용하는 개념
- redundancy는 wrist tracking과 별도로 controller가 풀도록 두는 구조

그대로 복사하지 않은 부분:

- H1_2 / H2 전용 IK solver
- Unitree 관절 제한 및 link length
- H1/H2의 고정 wrist offset, arm scale
- 해당 로봇 전용 regularization / smoothing 계수

AI Worker에서는 기존 Cyclo controller의 joint limit, collision, damping,
startup/reference guard를 그대로 사용한다.

## 4. AI Worker에서 추가한 부분

### Fixed calibration frame

캘리브레이션 시 Quest heading과 양쪽 wrist anchor를 고정한다.
활성 중 머리를 움직여도 이미 고정된 손목 목표 좌표계는 따라 돌지 않는다.

### Independent left/right wrist targets

좌우 손목을 각각 독립적으로 계산한다.
기존 human wrist-span 보정처럼 한쪽 손의 변화가 반대쪽 wrist goal을
강제로 이동시키지 않는다.

### Shoulder redundancy

AI Worker의 7-DoF arm redundancy는 사람 shoulder angle을 직접 복사하지 않는다.
wrist position/orientation을 만족하는 범위에서 기존 Cyclo QP가 joint motion을 결정한다.

필요한 경우 `SHOULDER_FREE.md`의 opt-in profile처럼 elbow position weight를
낮춰 shoulder roll을 포함한 관절 분배 자유도를 늘릴 수 있다.

### Stop / resume

실제 하드웨어 모드에서는 Y+B로 정지한 뒤 X+A를 누르면 현재 손과 측정된
로봇 손목을 다시 연결한다. 로봇 컨트롤러의 실제 활성화 대기와 램프가 끝나면
준비 중 움직인 손 자세를 다시 영점으로 잡고, 이후 상대 움직임부터 반영한다.
최종 손 영점 갱신은 유지 중인 로봇 목표, 좌표축과 팔꿈치/작업공간 기준을
바꾸지 않는다. `WAIT_START`에서 X+A를 눌러도 같은 방식으로 손 영점을 갱신한다.
`INPUT_ZERO`와 `READY` 로그를 확인한 뒤 움직인다.

SIM의 fixed-resume 정책은 `XR_FIXED_RESUME.md`를 따른다. 하드웨어의 최신
실행 순서와 피드백 대기는 [hardware/README.md](hardware/README.md)를 참고한다.

## 5. 현재 구현 위치

주요 구현은 다음 파일들에 있다.

```text
robotis_vuer/robotis_vuer/vr_publisher_sg2_xr.py
robotis_vuer/robotis_vuer/xr_arm_targets.py
robotis_vuer/robotis_vuer/vr_publisher_ai_worker_hardware.py
```

실행 시 AI Worker 쪽 기존 Python arm retargeter와 목표를 중복 발행하지 않도록
Cyclo launch는 `arm:=false`로 두고, XR publisher가 직접
`/[lr]_goal_pose` 계열 robot-frame target을 공급한다.

## 6. 한 줄 요약

**AI Worker XR retargeting은 사람 팔 관절을 복제하는 IK가 아니라,
Quest 손목의 상대 6D motion을 현재 AI Worker 손목 pose에 붙이고,
실제 관절 해는 기존 Cyclo QP에 맡기는 wrist-delta retargeting이다.**

Reference:

- `MFIWO/xr_tele` H1_2/H2 arm IK:
  `teleop/robot_control/robot_arm_ik.py`
- `MFIWO/xr_tele` AI Worker branch:
  `teleop/robot_control/robotis_ai_worker.py`
