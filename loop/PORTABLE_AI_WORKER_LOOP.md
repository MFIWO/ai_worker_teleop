# 다른 PC/AI Worker에 Loop 저장 구성을 이식하는 방법

이 문서는 이번에 `ffw-SNPR48A1111`과 Loop Desktop 1.1.4에 적용한 구성을
다른 Loop PC와 다른 ROBOTIS AI Worker에 재현하기 위한 절차다.

## 1. 구성과 역할

```text
AI Worker ROS topics
  ├─ /joint_states, /leader/* ── ffw_loop_streamer ── gRPC/TCP 50051 ──▶ Loop PC
  └─ camera */compressed ─────── ffw_loop_streamer ── RTSP 8554..8556 ◀─ Loop PC
                                                                      │
                                                             Preview / Recorder
```

- Loop Desktop은 PC에서 source를 미리 보고 저장하는 수신기다.
- `ffw_loop_streamer`는 AI Worker에서 ROS 데이터를 Loop source로 변환하는 송신기다.
- JPEG 재압축은 Loop Desktop이 아니라 AI Worker의 streamer에서 수행한다.
- Loop 앱만 실행해도 자동 연결되려면 AI Worker에서 streamer가 이미 실행 중이어야 한다.
  권장 실행 방법은 기존 `ffw_sg2_ai` 대신 통합 launch 하나만 사용하는 것이다.

## 2. 이번 코드 변경

| 파일 | 변경 내용 |
|---|---|
| `ffw_loop_streamer/loop_streamer.py` | 카메라별 JPEG 재압축 선택, q1~100 검증, RTP 전송 전 OpenCV 재인코딩 |
| `ffw_loop_streamer/loop_streamer_node.py` | `jpeg_recompress_cameras`, `jpeg_quality`, 카메라 회전 파라미터 연결 |
| `launch/loop_streamer.launch.py` | 위 파라미터와 `source_key`를 launch 인자로 노출 |
| `launch/ffw_sg2_ai_loop.launch.py` | follower/leader/camera 시작 후 streamer를 한 번만 실행 |
| `config/ffw_sg2_rev1.yaml` | 일반 물리 leader용 observation/action/camera topic 정의 |
| `config/ffw_sg2_vr.yaml` | 별도 VR 구성용이며 물리 leader 기록에는 사용하지 않음 |
| `test/test_loop_streamer.py` | 재압축 품질, RTP 호환성, 손목 회전 및 포트 구성을 검증 |

기존 구현은 RTP/JPEG와 호환되는 ROS `CompressedImage`를 그대로 통과시켰다.
따라서 `jpeg_quality`를 낮춰도 head 카메라 전송량이 바뀌지 않았다. 수정본은
기본적으로 `cam_head`만 q75로 재인코딩한다. 손목 카메라는 장착 방향 보정을 위해
이미 회전·재인코딩되므로 기본 재압축 대상에서 제외한다.

기본값은 다음과 같다.

```yaml
jpeg_quality: 75
jpeg_recompress_cameras: cam_head
camera_rotation: cam_wrist_left:90,cam_wrist_right:90
rotated_jpeg_quality: 90
```

`jpeg_recompress_cameras`를 빈 문자열로 지정하면 기존 JPEG pass-through로 돌아간다.
q75는 해상도를 바꾸지 않는다. Loop Preview에는 1280×720으로 보이면서 전송 JPEG
크기만 감소한다.

## 3. 이식 전에 정할 값

다른 장비에서는 아래 값을 현재 네트워크에 맞춰 정한다.

```text
LOOP_PC_IP     Loop Desktop이 실행되는 PC의 로봇망 IPv4 주소
ROBOT_IP       AI Worker의 로봇망 IPv4 주소
ROBOT_SSH      예: robotis@ffw-SERIAL.local
ROBOT_SOURCE   일반 leader는 robotis
ROBOT_PROFILE  일반 leader는 ffw_sg2_rev1
```

Loop PC에서 주소와 수신 포트를 확인한다.

```bash
ip route get <ROBOT_IP>
ss -ltnp | grep ':50051 '
```

Loop가 실행 중이면 `*:50051` 또는 `<LOOP_PC_IP>:50051`가 LISTEN 상태여야 한다.
AI Worker에서 `<LOOP_PC_IP>:50051`로 TCP 연결할 수 있어야 하며, Loop PC에서는
AI Worker의 TCP 8554~8556에 접근할 수 있어야 한다.

## 4. AI Worker에 패키지 배포

이 저장소에서 배포할 원본은 다음 디렉터리다.

```text
/home/kimm/Config/robotis-loop-integration/ffw_loop_streamer
```

대상 로봇의 기존 사용자 수정부터 확인한다.

```bash
ssh <ROBOT_SSH> 'git -C /home/robotis/ai_worker status --short'

rsync -avn \
  /home/kimm/Config/robotis-loop-integration/ffw_loop_streamer/ \
  <ROBOT_SSH>:/home/robotis/ai_worker/ffw_loop_streamer/
```

특히 대상 로봇에 별도 `camera_rotation`, topic 또는 launch 수정이 있으면 먼저
병합한다. 확인 없이 전체 디렉터리를 삭제하거나 `rsync --delete`를 사용하지 않는다.

대상 패키지를 백업한 뒤 동기화한다.

```bash
ssh <ROBOT_SSH> \
  'mkdir -p /home/robotis/ai_worker/docker/workspace/loop_backups && \
   tar -C /home/robotis/ai_worker -czf \
   /home/robotis/ai_worker/docker/workspace/loop_backups/ffw_loop_streamer.before_port.tar.gz \
   ffw_loop_streamer'

rsync -av \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '.pytest_cache' \
  /home/kimm/Config/robotis-loop-integration/ffw_loop_streamer/ \
  <ROBOT_SSH>:/home/robotis/ai_worker/ffw_loop_streamer/
```

컨테이너에서 테스트하고 빌드한다.

```bash
ssh <ROBOT_SSH>
docker exec -it ai_worker bash

source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}

cd /root/ros2_ws/src/ai_worker/ffw_loop_streamer
/usr/bin/python3 -m pytest -q test/test_loop_streamer.py test/test_vr_profile.py

cd /root/ros2_ws
colcon build --packages-select ffw_loop_streamer --symlink-install
source /root/ros2_ws/install/setup.bash
```

전체 테스트의 기대 결과는 `31 passed`다. `loop_sdk` 경로를 넣지 않으면 SDK 연동
2개가 skip될 수 있다.

`/home/robotis/ai_worker` 소스는 호스트 bind mount라 남지만, 컨테이너를 새로 만들면
컨테이너 내부의 build/install 결과와 `/opt/venv`가 사라질 수 있다. 대상 운영 방식에
따라 수정 이미지를 저장하거나 컨테이너 생성 후 빌드 단계를 자동화해야 한다.

## 5. Loop PC 주소 설정

AI Worker의 `/home/robotis/ai_worker/docker/.env`에서 기존 값을 보존하며 다음 항목을
현재 Loop PC 주소로 설정한다.

```dotenv
LOOP_ADDR=<LOOP_PC_IP>:50051
LOOP_ROBOT_TYPE=ffw_sg2_rev1
```

이미 실행 중인 컨테이너는 생성 당시 환경변수를 유지할 수 있다. 확인한다.

```bash
docker inspect ai_worker --format '{{range .Config.Env}}{{println .}}{{end}}' \
  | grep -E '^(LOOP_ADDR|LOOP_ROBOT_TYPE)='
```

환경변수가 다르더라도 launch의 `loop_addr:=<LOOP_PC_IP>:50051`로 명시할 수 있다.

## 6. Head 카메라를 1280×720으로 설정

AI Worker 호스트에서 다음 파일을 백업한다.

```text
/home/robotis/ai_worker/ffw_bringup/config/common/zedm.yaml
```

`general.grab_resolution`만 변경한다.

```diff
-            grab_resolution: 'VGA'
+            grab_resolution: 'HD720'
```

`grab_frame_rate: 30`은 유지한다. 설정은 ZED를 다시 열 때 적용되므로 실행 중인
통합 launch를 정상 종료한 후 다음 실행에서 반영한다. 해상도만 바꾸기 위해 실행 중인
모터 프로세스를 강제 종료하지 않는다.

적용 후 확인값은 다음과 같아야 한다.

```bash
ros2 param get /zed/zed_node general.grab_resolution
ros2 topic echo --once /zed/zed_node/left/camera_info sensor_msgs/msg/CameraInfo \
  | grep -E '^(height|width):'
```

```text
String value is: HD720
height: 720
width: 1280
```

## 7. 일반 물리 leader 기록 실행

follower, leader, 카메라와 streamer를 처음부터 함께 실행할 때 권장 명령은 하나다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:${PYTHONPATH:-}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=30

ros2 launch ffw_loop_streamer ffw_sg2_ai_loop.launch.py \
  loop_addr:=<LOOP_PC_IP>:50051 \
  fps:=30.0
```

이 통합 launch가 실행 중이면 별도 `loop_streamer.launch.py`를 다시 실행하지 않는다.
중복 실행하면 `/loop_streamer` 노드가 두 개 생기고 RTSP 8554 포트 충돌로 카메라가
비활성화된다.

기존 `ffw_sg2_ai`가 이미 실행 중일 때만 streamer를 별도로 실행한다.

```bash
ros2 launch ffw_loop_streamer loop_streamer.launch.py \
  robot_type:=ffw_sg2_rev1 \
  source_key:=robotis \
  loop_addr:=<LOOP_PC_IP>:50051 \
  rtsp_advertise_host:=<ROBOT_IP> \
  fps:=30.0 \
  jpeg_quality:=75 \
  jpeg_recompress_cameras:=cam_head
```

Loop Desktop에서 선택할 source는 다음과 같다.

```text
robotis
camera-cam-head
camera-cam-wrist-left
camera-cam-wrist-right
```

## 8. 검증

AI Worker에서 streamer가 하나인지 확인한다.

```bash
ros2 node list | grep -c '^/loop_streamer$'
```

결과는 `1`이어야 한다. 로그에는 다음 형태가 나와야 한다.

```text
subscribed camera cam_head ... (recompress q75)
camera 'cam_head' 1280x720 -> rtsp://<ROBOT_IP>:8554/cam_head
robot-step source connected to Config Loop at <LOOP_PC_IP>:50051
```

Loop PC에서 확인한다.

```bash
ss -tnp | grep ':50051'
timeout 3 bash -lc 'exec 3<>/dev/tcp/<ROBOT_IP>/8554'
```

Loop Preview의 head 설정은 `1280×720 @ 30fps`로 보여야 한다. UI에는 JPEG q75가
별도 표시되지 않으므로 q75 적용 여부는 AI Worker streamer 로그로 확인한다.

실제 전송량은 다음 식으로 비교한다.

```text
평균 JPEG bytes × 실제 FPS × 8 = camera payload bit/s
```

HD720 q75에서도 네트워크 여유가 부족하면 같은 장면으로 q70, q65를 순서대로 비교한다.
FPS를 낮출 때는 camera, streamer, Loop Recorder의 기록 Hz를 함께 조정한다.

## 9. 원복

1. 통합 launch를 `Ctrl-C`로 정상 종료한다.
2. 백업한 `zedm.yaml`과 `ffw_loop_streamer`를 복원한다.
3. `colcon build --packages-select ffw_loop_streamer --symlink-install`을 다시 실행한다.
4. `/loop_streamer`가 하나만 실행되는지와 Loop source key를 확인한다.

이번 `ffw-SNPR48A1111` 적용 백업은 다음 위치에 있다.

```text
/home/robotis/ai_worker/ffw_bringup/config/common/zedm.yaml.before_hd720_20260922.bak
/home/robotis/ai_worker/docker/workspace/loop_backups/ffw_loop_streamer.before_jpeg_recompress_20260922.tar.gz
```

이 경로는 현재 로봇의 기록이므로 다른 AI Worker에서는 장비별 백업을 새로 만든다.
