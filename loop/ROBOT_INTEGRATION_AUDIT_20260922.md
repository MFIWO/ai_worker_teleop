# AI Worker Quest → Config Loop 연결 및 ZED Mini 압축 조사

**후속 적용 완료 — 사용자 요청으로 head cam을 1280×720으로 변경했다.**

- 로봇 호스트 `/home/robotis/ai_worker/ffw_bringup/config/common/zedm.yaml`에서
  `general.grab_resolution`만 `VGA` → `HD720`으로 변경했다. 기존 30fps 및 JPEG 설정은 유지했다.
- 원본 백업: `/home/robotis/ai_worker/ffw_bringup/config/common/zedm.yaml.before_hd720_20260428T144027.bak`.
  백업 파일의 날짜는 로봇 시계를 따른다. 조사 시 로봇 시계는 PC와 달랐으며 시각 설정은 변경하지 않았다.
- ROS domain 30에서 head 카메라만 실행했다. 현재 이 실행을 유지한다.
  `docker exec ai_worker cat /tmp/ffw_head_hd720_20260922.pid`로 launch PID를 확인할 수 있다.
  로그는 컨테이너 `/tmp/ffw_head_hd720_20260922.log`에 있다.
- 실제 좌우 `/image_rect_color/compressed`를 각각 150프레임 구독하고 JPEG를 디코딩했다.
  양쪽 모두 **1280×720**, 좌 30.15fps / 우 30.18fps였다.
- 조회된 JPEG 품질은 좌우 모두 **95**였다. 정확한 파라미터 이름은
  `.zed_node.left.image_rect_color.compressed.jpeg_quality` 및
  `.zed_node.right.image_rect_color.compressed.jpeg_quality`다. 앞의 점도 이름에 포함된다.
- 측정 장면에서 평균 JPEG 크기는 좌 260,187바이트 / 우 261,397바이트였다.
  구독 수신률을 곱한 영상 payload는 각각 약 62.75 / 63.12Mbps다.
  이는 로봇 내부 측정이며 PC까지의 전송 지연이나 네트워크 전체 트래픽 실측은 아니다.
- Loop profile 설치와 실제 녹화는 이번 후속 요청에서 수행하지 않았다.

기존 전체 camera bringup을 실행하려면 이 단독 카메라 launch부터 정상 종료한다:

```bash
# 로봇 호스트에서 실행. 이번에 시작한 head 카메라 launch에만 SIGINT를 보낸다.
docker exec ai_worker bash -lc 'kill -INT "$(cat /tmp/ffw_head_hd720_20260922.pid)"'
```

이후 기존 bringup으로 실행해도 변경한 HD720 설정을 읽는다. 아래 내용은 **변경 전 최초 조사 기록**이며,
그 당시의 VGA·카메라 미실행·JPEG 품질 미확인 상태는 위 후속 적용 결과로 갱신됐다.

---

2026-09-22, `robotis@ffw-SNPR48A1112.local`에 SSH 접속하여 확인했다.
로봇 설정 변경, 설치, 카메라 실행, 모터 실행, 녹화는 수행하지 않았다.
첨부 G1 설명서는 비교 자료로만 읽었으며 그 안의 실행 명령은 실행하지 않았다.

**결론:** 로봇에 Loop 전송 프로그램과 SDK가 이미 설치돼 있다. 로컬의 Quest 전용 YAML을
로봇에 복사하고 기존 프로그램을 별도로 실행하면 된다. 로봇 제어 코드를 수정하거나
Loop UI를 로봇에 설치할 필요는 없다. ZED의 head 영상 경로는 이미 JPEG 압축을 사용한다.
화질·대역폭 조절은 ZED 발행 단계에서 해야 한다.

## 1. 실제 설치 상태

| 항목 | 확인 결과 |
|---|---|
| 로봇 IP | `192.168.6.2` |
| 로봇 OS/ROS | aarch64, Jetson 계열 / 컨테이너 ROS 2 Jazzy |
| 컨테이너 | `ai_worker`, 이미지 `robotis/ai-worker:1.2.1-loop` |
| ai_worker commit | `ae9b0b3`, 사용자 수정 및 추가 파일 있음 |
| 로봇 호스트 소스 | `/home/robotis/ai_worker` |
| 컨테이너 소스 | `/root/ros2_ws/src/ai_worker` — 위 경로의 bind mount |
| 영구 작업 파일 | 호스트 `.../ai_worker/docker/workspace` → 컨테이너 `/workspace` |
| Loop 패키지 | `ffw_loop_streamer` 설치 및 Python import 성공 |
| 실행 파일 | `ros2 run ffw_loop_streamer loop_streamer` |
| SDK | `/opt/venv`, `loop-sdk==0.4.1`, grpcio 1.83.1 |
| 로봇에 있는 profile | `ffw_sg2_rev1`, `ffw_bg2_rev4` |
| 아직 없는 profile | 로컬에 있는 `ffw_sg2_vr` |
| 기존 Loop 주소 | `192.168.6.56:50051` |
| 현재 작업 PC 로봇망 IP | `192.168.6.5` |
| 연결 검사 | 로봇→`.6.5:50051`: connection refused, `.6.56:50051`: timeout |
| 실행 상태 | 조사 당시 ZED/VR/Loop streamer 프로세스 없음; ROS domain 30 노드 목록 비어 있음 |

Loop 서버를 실제로 실행하는 PC의 주소를 사용해야 한다. 아래 예시는 현재 PC인
`192.168.6.5`를 사용한다. `.6.56`이나 다른 PC에서 녹화한다면 주소를 바꾼다.
현재 PC에서는 50051 수신 포트가 발견되지 않았으므로 Loop 서버 실행 및 외부 접속 설정을
먼저 확인해야 한다. 이 결과만으로 Loop 앱의 설치 여부까지 판단하지는 않았다.

## 2. 어디에 연결하는가

```text
Quest → PC robotis_vuer → 로봇 VR controller → follower
                              │                 │
                    최종 JointTrajectory    /joint_states
                              └────────┬────────┘
                                ffw_loop_streamer ── gRPC ──→ PC Loop 서버
                                       ↑                        │
ZED Mini → 로봇 JPEG /compressed ────────┘                        │
                                RTSP/RTP-JPEG ←── PC가 연결 ─────┘
                                                             Recorder 저장
```

- `observation`: `/joint_states`에서 읽은 실제 관절 위치.
- `action`: `/leader/joint_trajectory_command_broadcaster_{left,right}/joint_trajectory`.
  로봇 VR controller가 사용하는 발행 토픽 이름을 설치 소스에서 확인했다.
  `leader`라는 이름이 있어도 Quest 운용에서는 VR controller의 최종 발행 관절 명령이다.
- 로컬 `ffw_sg2_vr.yaml`은 양팔 각각 7관절, 그리퍼 각각 1관절을 분리한다.
  기존 리더 profile은 팔+그리퍼 8관절을 묶으므로 hand 모드에서 그리퍼가 누락되면
  팔 action까지 결측 처리될 수 있다. Quest에는 VR profile을 사용한다.
- UI 키 전달 코드 `robotis_vuer/loop_keys.py`는 PC의 Recorder 키 조작을 위한 것이다.
  로봇 데이터를 전송하는 역할은 `ffw_loop_streamer`가 담당한다.
- 이 구조에서 데이터 파일을 저장하는 주체는 Loop 서버/Recorder다.
  로봇의 디스크에 직접 저장하는 기능을 설치하는 것은 아니다.

ROS 데이터를 PC에서도 모두 구독할 수 있다면 PC에 streamer를 둘 수도 있다.
다만 현재 패키지와 SDK는 로봇에 이미 준비돼 있고, 카메라를 로봇 내부에서 구독해
RTSP로 전달하는 경로를 그대로 쓰는 편이 적용 범위가 작다.

## 3. 현재 로봇에 적용하는 최소 절차 — 아직 실행하지 않음

현재 설치된 node는 `source_key`와 YAML 파일을 직접 받을 수 있다.
**YAML 복사 + `ros2 run`이면 되며, 이 방식에는 소스 덮어쓰기·colcon 재빌드가 필요 없다.**

PC에서 설정 파일을 복사한다:

```bash
cd /home/kimm/Downloads/external_repos/loop
ssh robotis@ffw-SNPR48A1112.local \
  'mkdir -p /home/robotis/ai_worker/docker/workspace/loop_vr_20260922'
scp loop/ffw_loop_streamer/config/ffw_sg2_vr.yaml \
    loop/deployment/zedm_hd720.yaml \
    robotis@ffw-SNPR48A1112.local:/home/robotis/ai_worker/docker/workspace/loop_vr_20260922/
```

기존 follower, Quest teleop 및 카메라가 준비된 뒤 로봇 컨테이너의 별도 터미널에서:

```bash
ssh robotis@ffw-SNPR48A1112.local
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30

ros2 run ffw_loop_streamer loop_streamer --ros-args \
  --params-file /workspace/loop_vr_20260922/ffw_sg2_vr.yaml \
  -p robot_type:=ffw_sg2_vr \
  -p source_key:=robotis-vr \
  -p loop_addr:=192.168.6.5:50051 \
  -p fps:=30.0 \
  -p camera_probe_timeout_s:=120.0 \
  -p rtsp_advertise_host:=192.168.6.2
```

Loop에서 `robotis-vr`와 `camera-cam-head`, `camera-cam-wrist-left`,
`camera-cam-wrist-right`를 선택하고 30Hz로 기록한다.
로봇→PC TCP 50051, PC→로봇 TCP 8554~8556 연결이 필요하다.
카메라 준비가 늦어 120초 안에 프레임이 없으면 해당 카메라는 이번 실행에서 제외되므로
카메라 준비 후 streamer를 다시 시작한다.

이 명령은 로봇 topic을 구독하는 프로그램만 시작한다. follower나 VR 제어기는 따로
기존 절차로 실행한다. 물리 리더까지 함께 켜는 `ffw_sg2_ai_loop` 통합 launch는
Quest 구성의 시작 명령으로 사용하지 않는다. 기존 streamer와 중복 실행하지 않는다.

파일은 `/workspace`의 bind mount에 남지만 streamer 프로세스는 컨테이너 재시작 시
다시 실행해야 한다. 위 `ros2 run`에는 자동 재시작 기능이 없다.

### 기존 installer를 사용할 경우

`loop/scripts/install_vr_profile_in_container.py`를 로봇에서 복사한 소스의 **로컬 임시 복사본**에
실행해 보았다. VR YAML 추가, launch 인자 추가, 재실행 모두 성공했고 손목 회전 옵션도 유지됐다.
단, 새 launch의 기본 `source_key`를 `robotis`로 지정한다. 현재 로봇 node의 기본값은
`robot-step`이므로 기존 소스 이름이 그대로 보존된다는 스크립트 안내와 실제 동작이 다르다.
기존 UI의 소스 선택에 영향을 줄 수 있어 위의 명시적 `ros2 run`을 우선 제안한다.

로컬 전체 패키지를 로봇에 덮어쓰지 않는다. 로봇에는 로컬과 다른
`camera_rotation`, `rotated_jpeg_quality=90` 구현이 있고, 현재 손목 회전 처리를 보존해야 한다.

## 4. ZED Mini의 실제 영상 경로

로봇의 `ffw_bringup/launch/camera_zed.launch.py`는 **ffw_bringup의**
`config/common/zedm.yaml` 및 `common_stereo.yaml`을 읽는다.
`zed_wrapper/config/zedm.yaml`에 있는 HD1080 설정은 이 bringup의 기본 설정이 아니다.
설치 디렉터리도 해당 ffw_bringup 소스로 연결돼 있음을 확인했다.

현재 bringup 설정 파일 값:

```yaml
general.grab_resolution: VGA
general.grab_frame_rate: 30
general.pub_resolution: NATIVE
general.pub_downscale_factor: 1.0
general.pub_frame_rate: 30.0
depth.depth_mode: NONE
pos_tracking.pos_tracking_enabled: false
stream_server.stream_enabled: false
```

카메라가 실행 중이지 않으므로 이는 파일 설정이며, 실측 해상도·FPS는 아니다.

```text
ZED → image_transport JPEG → /zed/zed_node/left/image_rect_color/compressed
    → Loop streamer set_jpeg → JPEG 파싱·RTP 패킷화 → RTSP/TCP → Loop
```

Head JPEG가 RTP/JPEG와 호환되면 디코딩·재인코딩 없이 전달한다.
따라서 Loop node의 `jpeg_quality=80`을 낮춰도 이 head 경로의 전송량은 줄어들지 않는다.
그 값은 비호환 JPEG의 재인코딩 등 fallback에 쓰인다. 손목은 회전을 위해 재인코딩한다.

설치된 `compressed_image_transport`는 4.0.6이다. 같은 버전의 공식 소스에서
JPEG 기본 품질 95와 토픽별 `compressed.jpeg_quality` 파라미터를 확인했다.
그러나 현재 로봇의 실행 중 품질값을 읽은 것은 아니므로 **실제 운용값이 95라고 확정하지 않는다.**
[공식 4.0.6 소스](https://github.com/ros-perception/image_transport_plugins/blob/4.0.6/compressed_image_transport/src/compressed_publisher.cpp)

## 5. 제안: HD720·30fps + ZED JPEG 품질 75부터 비교

`deployment/zedm_hd720.yaml`에 해상도 override를 준비했다. 품질 75는 성능을 보장하는
값이 아니라 비교 측정을 시작할 후보값이다. YAML 자체에는 JPEG 품질을 넣지 않았다.
정확한 파라미터 접두어는 실제 노드에서 조회해 사용한다.

카메라가 꺼진 상태에서 로봇 컨테이너에 위 ROS 환경을 설정하고:

```bash
ros2 launch ffw_bringup camera_zed.launch.py \
  camera_model:=zedm \
  ros_params_override_path:=/workspace/loop_vr_20260922/zedm_hd720.yaml
```

이 명령은 ZED 카메라만 시작한다. 기존 카메라 launch가 이미 실행 중이면 중복 실행하지 않고
그 실행 경로에 같은 override를 전달하도록 구성한다. 손목 카메라는 기존 실행 절차를 사용한다.
촬영 해상도 변경은 카메라를 다시 열어야 하므로 녹화 에피소드 사이에 적용한다.

ZED 실행 후 다른 컨테이너 터미널에서 파라미터를 찾는다:

```bash
ros2 param list /zed/zed_node | grep -E 'image_rect_color.*jpeg_quality'
```

그 결과의 **left/right 각각 `compressed.jpeg_quality`로 끝나는 전체 이름**을 사용한다:

```bash
# 아래 문자열은 자리표시자다. 바로 위 조회에서 나온 이름으로 바꾼다.
ros2 param get /zed/zed_node '<left의 전체 파라미터 이름>'
ros2 param set /zed/zed_node '<left의 전체 파라미터 이름>' 75
ros2 param get /zed/zed_node '<right의 전체 파라미터 이름>'
ros2 param set /zed/zed_node '<right의 전체 파라미터 이름>' 75
```

변경 전 값을 보관하고 `get`으로 변경 성공을 확인한다. Loop만 기록하면 left가 대상이고,
Quest 양안 영상까지 사용하면 right도 대상이다. 런타임 변경은 재시작하면 사라지므로,
성능 확인 후 정확한 파라미터 이름을 같은 override YAML의 `ros__parameters` 아래에
정수 `75`로 추가하면 재현할 수 있다. 완전한 `param dump`를 그대로 덮어씌우지는 않는다.

원복은 streamer를 종료하고, ZED를 override 인자 없이 기존 방식으로 다시 시작하면 된다.
기존 카메라 YAML과 Docker 환경은 위 절차에서 편집하지 않는다.

비교 순서는 VGA 현재 품질 → HD720 q75 → HD720 q80/85이다. CPU가 충분하고 대역폭에
여유가 있으면 HD1080도 후보가 된다. 현재 RTP/JPEG 구현은 각 변 2040픽셀까지만 지원하므로
단안 HD720/HD1080은 범위 안이지만 HD2K 2208픽셀 폭이나 HD720 양안을 붙인 2560픽셀 폭은
그대로 사용할 수 없다. G1 첨부 문서의 자동 좌우 분리 기능은 이 AI Worker 구현에 없다.

FPS를 낮출 때는 ZED, streamer, Loop의 기록 주기를 함께 검토한다.
카메라만 15fps로 낮추고 streamer를 30fps로 두면 같은 프레임을 반복 전송할 수 있다.
전송량은 `평균 JPEG 바이트 × 실제 전송 FPS × 8`로 먼저 계산하고 프로토콜 overhead를 더한다.
예를 들어 100kB × 30fps는 영상 하나당 약 24Mbps이며 실제 JPEG 크기는 장면마다 달라진다.

## 6. Quest 화면 지연과 Loop 기록 지연은 별도 경로

로컬 hand publisher `vr_publisher_sh5.py`는 선택적으로 양쪽 `/compressed`를 구독하고,
JPEG bytes를 Vuer `ImageBackground`에 전달한다. 기본 `enable_vr_image=false`,
표시 주기는 `vr_image_fps=15.0`이다. `vr_publisher_hand_neck.py`도 이 기반을 사용한다.
SG2 controller publisher에는 같은 영상 구독 구현이 확인되지 않았다.

따라서 Quest 표시를 켠 hand 구성에서는 로봇→PC DDS 양안 영상과 로봇→Loop head RTSP가
동시에 네트워크를 쓸 수 있다. Loop의 head는 left 한 채널이며 양안 전체 기록은 아니다.
Quest 표시 주기만 낮춰도 로봇→PC의 DDS 발행량까지 자동으로 줄어들지는 않는다.
현재 PC의 teleop 실행 상태는 확인되지 않아 실제 표시 경로가 켜져 있다고 가정하지 않았다.

JPEG 튜닝 후에도 고해상도 전송이 부족하면 H.264/H.265 하드웨어 인코딩을 검토할 수 있다.
ZED SDK는 이를 지원하지만 실제 Orin 보드의 인코더 가용성과 처리량은 추가 확인이 필요하다.
현재 `set_jpeg` 및 RTP/JPEG 코드는 H.264를 받을 수 없으므로 코덱 이름만 바꾸면 안 된다.
SDK 수신 브리지 또는 H.264 RTSP 서버/Loop 소스 연결을 별도로 구현하고 검증해야 한다.
ZED SDK 스트림도 현재 Loop의 RTSP 주소로 그대로 대체할 수 있는 것으로 확인된 것은 아니다.
[Stereolabs 하드웨어 전송 브리지](https://docs.stereolabs.com/docs/integrations/ros-2/hardware-encoding-bridge),
[SDK H.264/H.265 전송](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/local-network-streaming)

## 7. 검증 범위와 운용 시 확인할 사항

- 로봇의 실제 mapper 코드로 로컬 VR profile 기존 테스트 4개 통과.
  양팔 7관절, 그리퍼 분리, 누락 action을 observation으로 채우지 않는 동작을 확인했다.
- 실제 로봇 launch 복사본에 기존 installer를 적용하고 반복 실행을 검증했다.
  로봇 파일 자체에는 적용하지 않았다.
- 로봇 컨테이너의 설치된 node와 SDK import 성공, 실행 파일 존재를 확인했다.
- 카메라가 꺼져 있고 Loop 서버도 접속되지 않아 실제 대역폭·지연·화질·저장 에피소드는 미검증이다.

실행 후에는 같은 장면으로 짧은 에피소드를 만들고, head 해상도/FPS, 평균 프레임 크기,
CPU 부하, Quest 표시 지연, Loop 저장 영상의 프레임 수를 비교한다.
카메라 토픽의 `ros2 topic info -v`로 원격 raw 구독자가 있는지도 확인한다.
`ros2 topic bw`를 PC에서 추가 실행하면 그 측정 구독 자체가 전송량을 늘릴 수 있으므로
기존 연결 수와 측정 위치를 함께 기록한다.

현재 streamer는 최신 메시지를 캐시해 30Hz로 내보낸다. 원래 센서 timestamp에 맞춰
state/action/영상이 엄밀하게 동기화되는 구조는 아니다. 입력이 멈춰도 마지막 action이나
영상이 반복될 수 있고, freshness timeout 및 tracking 상태 채널이 없다.
VR profile에는 head/lift/mobile action 채널도 없으므로 이 동작을 학습에 포함하려면
실제 명령 토픽에 맞춘 profile 확장이 필요하다.

첨부 G1 문서의 JPEG60→ZMQ→PC 재인코딩 JPEG80 구조는 이 로봇의 현재 구조와 다르다.
AI Worker에서는 ROS JPEG를 로봇에서 바로 Loop로 보내는 기존 경로를 활용할 수 있다.

일반 인터페이스 설명은 [ROBOTIS 공식 ai_worker](https://github.com/ROBOTIS-GIT/ai_worker),
압축 및 네트워크 동작은 [Stereolabs ROS 2 문서](https://docs.stereolabs.com/docs/integrations/ros-2/dds-and-network-tuning)를
교차 확인했으며, 위 설치 상태와 적용 경로는 실제 로봇에서 읽은 소스를 기준으로 한다.
