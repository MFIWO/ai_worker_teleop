# Hand VR + Loop 통합 조작 터미널

원본 optical hand와 목 착용 hand의 실행 스크립트에 `operator_hand.py` 래퍼를 연결했다.
원본 `vr_publisher_sh5.py` 및 팔 좌표 변환 수식은 변경하지 않았다.
`xr_tele`의 U 정지에서 팔·손 입력을 보류하는 개념을 가져왔으며 해당 프로젝트의
페달 프로세스, UDP E-stop, tracking heartbeat와는 연결하지 않는다.
현재 SG2 controller 서버에는 이 hand 전용 래퍼를 적용하지 않는다.

## 실행 순서

1. Loop 앱을 열고 평소 사용하는 Recorder 화면/소스/카메라를 준비한다.
2. 기존 PC hand 서버와 로봇 VR 제어기를 종료하고 follower만 유지한다.
3. PC 컨테이너에서 기존 `hand_neck_pc.sh` 또는 `hand_original_pc.sh`를 실행한다.
   **이 스크립트는 실제 양팔을 팔꿈치 90도 준비 자세로 움직인다.**
4. READY 이후 기존 `hand_original_robot.sh`로 로봇 VR 제어기를 실행한다.
   처음 활성화는 기존 pinch+반대손 주먹 제스처를 사용한다.
5. 기존 실행 터미널들은 유지하고, PC 호스트의 조작용 터미널에서 다음을 실행한다.

```bash
bash /home/kimm/Downloads/external_repos/robotis_applications/operator_console.sh
```

새 clone에서는 위 경로를 바꾸면 된다. 별도 ROS 패키지 빌드 없이 소스에서 실행한다.
`robotis-applications` 컨테이너와 기존 소스 mount를 사용한다.
`QUEST_PC_CONTAINER` 환경 변수로 다른 컨테이너 이름을 지정할 수 있다.

이 호스트 스크립트는 현재 데스크톱의 X11 인증 파일을 해당 컨테이너의 임시 파일로
복사하고 컨테이너 안의 조작기를 시작한다. X 서버 전체 접근을 허용하지 않는다.
현재 Ubuntu X11 세션용이며, Wayland 전용 세션이나 별도 SSH 화면용이 아니다.
키 입력은 이 터미널이 포커스를 가진 동안에만 처리한다.

읽기 전용 환경 점검:

```bash
bash /home/kimm/Downloads/external_repos/robotis_applications/operator_console.sh --check
```

일반 Linux 키보드 이벤트에서 key-down/key-up을 읽는다. `event-kbd` 장치를 자동으로
열며 `--device /dev/input/eventN`을 반복 지정할 수도 있다.
A/B/C/U를 보내도록 이미 설정된 USB 페달은 그대로 사용할 수 있다.
`FootSwitch` 키보드 장치의 A/B/C는 U 정지 중에도 항상 Loop로 전달해 목 A와 분리한다.
다른 이름의 페달은 `--loop-device /dev/input/eventN`으로 지정한다.
페달에 새 키를 설정하거나 전용 드라이버를 실행하는 기능은 포함하지 않는다.
여러 조작기 동시 실행은 컨테이너 내 lock으로 차단한다.

## 키 배치

| 키 | 동작 |
|---|---|
| U | 팔·손 목표 전달 일시정지 / 재개 요청 |
| ↑ / ↓ | 정지 중 mobile 전진 / 후진 |
| ← / → | 정지 중 mobile 좌 / 우 평행 이동 |
| Q / E | 정지 중 mobile Z축 좌 / 우 회전 |
| W / S | 정지 중 목 위 / 아래 tilt |
| A / D | 정지 중 목 좌 / 우 pan |
| O / P | 정지 중 lift 위 / 아래 |
| 키보드 A / B / C | Loop 키 전달. **정지 중 키보드 A는 목 조작** |
| 페달 A / B / C | 정지 여부와 무관하게 Loop 전달 |
| Shift+A | 정지 중에도 Loop A 전달 |
| Space | 수동 이동 키 상태 초기화 및 이동 정지 |
| Esc / Ctrl+C | 이동 정지, VR 일시정지 요청 후 조작기 종료 |

키 자동 반복은 U와 Loop 동작을 반복 실행하지 않는다. 누름/해제는 Loop에 그대로
전달하므로 C의 짧게/길게 누르기 의미도 유지한다. Loop A/B/C의 실제 역할은
Loop 화면 상태에 따른 기존 동작을 그대로 따른다. Recorder 창을 연 뒤
입력란에서 벗어난 상태로 사용한다. Loop 창이 없거나 여러 개면 전달 오류를 표시한다.
키 전송은 Loop 창의 X11 이벤트에 한정하며 실제 활성 창은 조작 터미널에 유지한다.
Loop 앱/서버를 자동 실행하거나 녹화 설정을 바꾸지는 않는다.

수동 속도: mobile 0.15m/s, 회전 0.3rad/s, 목 0.25rad/s, lift 0.04m/s.
SG2 URDF 한계: 목 tilt [-0.2317, 0.6951], pan [-0.35, 0.35] rad,
lift [-0.5, 0.0]m. 관절 목표는 신선한 측정값에서 시작하고 측정값보다 크게 앞서
누적되지 않도록 제한한다. 다른 로봇 모델에는 해당 모델의 한계 확인이 필요하다.
키 해제, 포커스 이탈, 입력 장치 끊김 및 VR 상태 수신 만료 시 이동 입력을 제거한다.
프로세스 강제 종료 시 base 정지는 기존 follower의 cmd_vel timeout에도 의존한다.

## U의 의미와 재개

U는 소프트웨어 추종 일시정지다. 팔·손의 새 목표와 finger retargeting 입력을 막고
`/reactivate=false`로 로봇 VR 계산 출력을 해제한다. 팔은 마지막 전달 목표까지
수렴할 수 있으며 토크 해제나 물리 비상정지가 아니다. 손에 zero/open 명령을 보내지 않는다.
정지 중 손 제스처로 이 상태를 해제할 수 없고 U로 다시 요청해야 한다.

다시 U를 누르면 양손의 신선한 입력과 기존 허용 목표와의 연속성을 검사한다.
멈췄던 목표 근처(5cm/15도 이내)에서 약 0.3초 안정된 뒤 재개한다.
목 착용 경로의 기존 추적 guard 안정화 시간이 추가될 수 있다.
손을 멀리 옮겨둔 채 U만 눌러 로봇이 그쪽으로 갑자기 움직이게 하지는 않는다.
터미널의 `return_to_held_pose`는 손을 멈췄던 목표 근처로 돌려놓으라는 뜻이다.
로봇 startup/reference 검사도 유지하므로 재개 시 기존 활성화 대기가 발생할 수 있다.
리프트를 크게 이동했으면 로봇 실제 목표 높이가 달라져 startup 조건에 맞게 손 높이를
다시 맞춰야 할 수 있다. 임의 좌표 오프셋이나 사용자 팔 길이 보정을 추가하지 않는다.
처음부터 VR 비활성이었다면 U 해제만으로 처음 활성화하지 않으며 제스처가 필요하다.

## 데이터와 검증 범위

기존 Loop streamer를 그대로 사용한다. 이 터미널은 UI 키만 전달하며 별도의 recording
클라이언트를 만들지 않는다. 현재 팔 전용 VR profile은 mobile/head/lift의 action을
기록하지 않는다. 정지 중에도 Loop의 캐시된 팔 action이 저장될 수 있다.
이 조작들을 학습 action에 포함하려면 Loop profile에 별도 채널을 추가해야 한다.

격리 ROS domain 231에서 모의 publisher와 실제 hand/body callback을 사용하는
새 테스트 9개와 기존 회귀 테스트 31개가 통과했다. 실제 모터 동작이나
실제 Loop 에피소드 생성은 테스트하지 않았다.
Qt WebEngine 모의 창에서는 다른 창의 포커스를 유지한 키 누름/해제를 확인했다.
실제 Loop Recorder, 하드웨어 이동 및 재개는 사용자 운용에서 추가 확인이 필요하다.

```bash
# PC 컨테이너 내부: 격리 domain을 스크립트가 설정한다.
bash /root/ros2_ws/src/robotis_applications/robotis_vuer/test/run_operator_tests.sh
```
