# Quest SG2 controller teleop: arm coordinate backport

2026-09-18, robot `ffw-SNPR48A1112` (`192.168.6.2`).

## 저장소 구성

| 브랜치 | 역할 |
|---|---|
| `main` | 로봇 `cyclo_control` 적용/복구 파일, 실행 문서, Loop VR 기록 설정 |
| [`robotis-applications`](https://github.com/MFIWO/ai_worker_teleop/tree/robotis-applications) | PC `robotis_applications` 전체 소스와 Quest 수정 사항 |

두 브랜치는 서로 다른 컴포넌트이므로 각각 별도 디렉터리에 clone한다.
같은 작업 폴더에서 브랜치를 전환하거나 두 브랜치를 merge하는 구조가 아니다.

```bash
git clone --branch main --single-branch https://github.com/MFIWO/ai_worker_teleop.git quest_sg2_teleop
git clone --branch robotis-applications --single-branch https://github.com/MFIWO/ai_worker_teleop.git robotis_applications
```

PC 소스는 기존 컨테이너의 `/root/ros2_ws/src/robotis_applications`에 연결되어야 한다.
새 폴더에 clone하는 것만으로 실행 중인 컨테이너의 mount가 바뀌지는 않는다.
이 문서의 IP, 호스트명, 경로는 기존 장비 기준이므로 다른 환경에서는 바꾼다.
로봇 설치 묶음은 컨테이너의 `/workspace/quest_sg2_teleop`에 복사하여 사용한다.
인증서와 개인 키는 포함하지 않는다. Quest가 접근하는 현재 서버 IP를 포함해
인증서를 별도로 생성하고 기기의 신뢰 설정을 완료해야 한다.

- 원본 optical hand 비교: [HAND_ORIGINAL.md](HAND_ORIGINAL.md)
- 목 착용 optical hand: [HAND_NECK.md](HAND_NECK.md)
- VR 팔 action과 카메라 기록: [loop/VR_LOOP.md](loop/VR_LOOP.md)
- 원본 코드 및 수정 범위: [SOURCES.md](SOURCES.md)

Loop VR profile은 로컬 변환 테스트까지 완료했으며 로봇 설치와 새 VR 실녹화는 미완료다.
원본 측정 로그, 영상, 인증서와 개인 키는 업로드하지 않는다.

## 변경 이유와 범위

PC의 `robotis_vuer`는 사람의 머리 기준으로 변환한 손목/팔꿈치/어깨 pose를
발행한다. 로봇의 `cyclo_control` 커밋 `651aa55`에는 이 입력을 로봇의 어깨
위치와 팔 길이에 맞추는 arm retargeting 노드가 없다. 손목 토픽을 goal에
직접 remap하면 입력은 도착하지만 사람의 팔 좌표를 그대로 로봇 목표로 써서
startup 위치 오차가 발생한다.

공식 `ROBOTIS-GIT/cyclo_control` 커밋
`b1b2033965e654cab252d4b68b1719e7e6d65b80`의
`cyclo_motion_controller_ros_py/scripts/arm_retargeting.py`를 그대로 백포트했다.

- `controller_type:=vr`에서 팔 변환 노드를 자동 실행한다.
- VR 제어기의 elbow 입력은 보정된 `/[lr]_subgoal_pose`에 연결한다.
- 기존 physical leader의 elbow 경로는 유지한다.
- `hand:=false`를 유지한다. 손가락 retargeting이나 혼합 입력 모드는 추가하지 않는다.
- startup 오차 제한과 관절 제한은 변경하지 않는다. VR 제어기의 그리퍼 명령 전달은 아래와 같이 수정한다.
- PC SG2의 그리퍼 발행 QoS는 별도 수정으로 Reliable이다. 로봇의 구버전
  raw trajectory subscriber와 호환된다.
- 로봇 VR 제어기의 최종 팔 trajectory에 수신한 그리퍼 관절 목표를 포함한다.
  이전 버전은 raw trigger를 저장하고도 최종 명령에서 그리퍼를 누락했다.
  측정 joint_states에 해당 그리퍼가 있고 유효한 trigger 목표를 수신했을 때만
  추가한다. 처음부터 임의의 개폐 명령을 만들지 않는다.

`original/`은 로봇의 수정 전 파일, `files/`는 적용 파일이다.
`install_backport.py`는 기존 파일이 original과 일치하는지 모두 검사한 후 적용한다.
로컬 묶음은 `/home/kimm/Config/quest_sg2_teleop`, 로봇의 영구 복사본은
`/home/robotis/ai_worker/docker/workspace/quest_sg2_teleop`이다.
컨테이너에서는 `/workspace/quest_sg2_teleop`로 보인다.

## 정상 실행

팔꿈치 90도 준비 자세를 포함한 실행은 아래의
[Controller 모드 시작 절차](#controller-모드에서-팔꿈치-90도-준비-자세로-시작)를 따른다.
follower만 실행한 상태에서 PC `controller_pc.sh`를 실행하고, READY 출력 후
로봇 VR 제어기를 시작한다. PC 스크립트는 실제 양팔을 움직인다.

Quest 페이지를 새로고침하고 VR에 진입한다. X/A에서 손을 뗀 상태로 추적이
잡히면 X+A로 활성화를 요청한다. 이후 Grip과 X/A를 계속 누를 필요가 없다.
Y+B로 팔 제어를 해제한다. body 또는 어느 한쪽 controller 추적이 0.5초 이상
끊기면 팔 제어를 해제하며, 재시작하려면 X/A를 놓았다가 다시 눌러야 한다.
실제/목표 자세의 시작 조건은 그대로 적용된다. 트리거는 SG2 그리퍼를 제어한다.
controller에서는 `/[lr]_wrist_pose`를 goal에 직접 remap하지 않는다.

## 어깨 yaw 진단

사용자 보고: 양쪽 어깨 yaw가 약 60도 외회전하고 roll/pitch는 정상.
이 수치는 아직 관절 데이터로 검증하지 않았으며 고정 60도 보정은 적용하지 않았다.
URDF에서 `arm_[lr]_joint1/2/3` 축은 각각 pitch/roll/yaw이다.
팔 변환은 사람 팔의 방향을 정규화하고 로봇 링크 길이를 사용한다.
팔 길이 자체를 사용자마다 수동 설정할 필요는 없지만 controller 손목 추정과
양손 거리 보정, 팔꿈치/손목 목표의 IK 절충은 별도 확인이 필요하다.

VR 제어 중 로봇 ai_worker 컨테이너의 별도 터미널에서 아래 명령을 실행하면
실제 관절각, 명령 관절각, 어깨/팔꿈치/손목 입력과 변환 목표를 30초 기록한다.
출력 각도 단위는 도이며 어떤 제어 명령도 발행하지 않는다.

```bash
source /root/ros2_ws/install/setup.bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 \
  python3 /workspace/quest_sg2_teleop/observe_shoulder_yaw.py
```

결과: `/tmp/quest_shoulder_yaw.json`. 손목 방향과 어깨 yaw 관절각은 다른 값이므로
손목 yaw offset을 어깨 보정값으로 바로 대입하지 않는다.

## 컨테이너 재생성 후 재적용

`cyclo_control` 소스/빌드 경로는 이 로봇에서 host bind mount가 아니므로
컨테이너 재생성 시 백포트를 다시 적용해야 한다. 백업 묶음은 host에 남는다.
실행 중인 VR 제어기를 종료한 상태에서 ai_worker 컨테이너 안에서:

```bash
python3 /workspace/quest_sg2_teleop/install_backport.py
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select cyclo_motion_controller_ros_py
colcon build --symlink-install --packages-select cyclo_motion_controller_ros \
  --cmake-target vr_controller_node --parallel-workers 1
```

## 검증

- 로봇 안에서 Python 패키지 빌드 성공.
- `test_backport.py`: 측정 샘플 위치 오차, 로봇 팔 길이, 원점 이동 불변성,
  프레임 시간 불일치/누락/퇴화 입력 거부, VR/leader launch 연결 6개 검사.
- 기존 측정 샘플의 보정 후 오차: 왼팔 0.2157m, 오른팔 0.2017m.
- `preview_backport.py`: 기존 direct-remap 서버를 입력으로 사용하며 모든
  출력은 `/quest_preview/`로 보낸다. 정상 운용용 launch와 함께 쓰지 않는다.
  결과는 `preview_result.json`에 저장한다.
- 테스트/미리보기에서 로봇의 활성화 명령이나 모터 명령을 발행하지 않는다.
- `test_gripper_forwarding.py`: `ROS_DOMAIN_ID=231`에서 설치된 VR 제어기에
  가상 joint_states/goal/trigger를 넣어 최종 8관절 명령을 검사한다.
  실제 로봇의 domain 30과 분리된다. trigger 입력 전 임의 개폐 명령 없음,
  양쪽의 독립적인 open/partial/close 명령 전달을 통과했다.

## 차렷 자세 재현의 남은 문제

사용자 관찰: 차렷 자세에서 로봇 팔꿈치가 몸통에서 떨어지고 굽혀지며,
오른쪽이 더 외회전하고 들린다. 목표 추종 오차가 작다는 사실만으로
사람의 자세가 올바르게 변환됐다고 결론 내릴 수 없다.

현재 모델의 양쪽 `arm_[lr]_joint4` 상한은 -0.7rad(-40.1도)이다.
그래서 팔꿈치를 더 펴는 명령은 제약에 걸린다. 이 제한을 임의로 풀지 않았다.
양쪽 제한은 같으므로 오른쪽이 더 심한 비대칭은 이 제한만으로 설명되지 않는다.
PC의 어깨 기준점은 scapula이고, controller로 추정한 손목에는 12cm 역방향 이동과
어깨/팔꿈치보다 10cm 낮은 Z 오프셋이 있다. 원시 body/controller 위치와 실제
차렷 자세의 비교가 필요하다. 고정 yaw 보정이나 roll/pitch 보정은 적용하지 않았다.

## Controller 모드에서 팔꿈치 90도 준비 자세로 시작

기존 PC VR 서버와 로봇 VR 제어기를 종료하고 follower만 유지한다.
PC robotis-applications 컨테이너:

```bash
bash /root/ros2_ws/src/robotis_applications/controller_pc.sh
```

기존 검증된 prepare_sg2_ready_pose.py로 양쪽 팔 관절을
[0, 0, 0, -1.57, 0, 0, 0] rad에 약 5초 동안 이동하고 실제 도달을 확인한 뒤
model:=sg2 VR 서버를 시작한다. 준비 동작 실패 시 VR 서버를 시작하지 않는다.
READY 출력 후 로봇 ai_worker 컨테이너에서:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr hand:=false
```

Quest 페이지를 새로고침하고 컨트롤러로 X+A 활성화, Y+B 정지, 트리거 그리퍼 개폐.
목 착용 hand 전용 좌표 보정/추적 보류 처리를 controller 경로에 추가한 것은 아니다.
