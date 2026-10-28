# AI Worker SH5 VR · Loop integration

SH5 팔·손 VR Teleop과 Loop 녹화를 함께 배포하는 `main` 브랜치입니다.
실행 명령과 다른 PC의 설치·업데이트 절차는
[ai_teleop_hand.txt](docs/ai_teleop_hand.txt)에 있습니다.
문서 위쪽에는 실행 명령, 아래쪽에는 디버깅과 설정·업데이트 명령을 모았습니다.

## 실행

PC 호스트의 터미널 두 개에서 실행합니다. 예시 주소는 PC `192.0.2.10`,
로봇 `192.0.2.20`은 문서용 예시이므로 실제 IP로 바꾸세요. `--loop vr`이면 PC에서 Loop 앱을 먼저 여세요.

터미널 1:

```bash
bash ~/Downloads/external_repos/ai_worker_teleop_deploy/teleop.sh robot \
  --pc-ip 192.0.2.10 --robot-ip 192.0.2.20 \
  --loop vr --input hands --hand-mode 1 --bringup
```

터미널 2:

```bash
bash ~/Downloads/external_repos/ai_worker_teleop_deploy/teleop.sh pc \
  --pc-ip 192.0.2.10 --robot-ip 192.0.2.20 \
  --loop vr --input hands --hand-mode 1
```

Quest 접속: `https://192.0.2.10:8012?ws=wss://192.0.2.10:8012`.
두 명령의 input과 hand-mode를 같게 설정하세요. Loop 없이 사용하려면 둘 다
`--loop off`로 바꿉니다. `--bringup`은 follower가 없으면 시작하고 팔만 5초 동안
준비 자세로 이동합니다. 손 초기화 궤적은 보내지 않습니다.

| 입력 | mode 1 | mode 2 |
|---|---|---|
| `--input hands` | 설치된 Cyclo 원본 solver/filter | xr_ai_teleop HX5 DexPilot, 엄지 MCP/IP 굽힘 반영 |
| `--input controllers` | Trigger grasp / Grip pinch, 속도 제한 | 버튼으로 포즈 선택 + 각 Trigger 비례 동작 |

맨손 입력은 HAND_MOVE에서 좌우 독립으로 즉시 처리합니다. mode 2는 최신 샘플을
사용하고, 엄지 joint3/4를 추적 굽힘으로 고정한 18관절 최적화를 수행합니다.
광학 손 mode 1/2에는 컨트롤러 mode 1의 0.85 속도 제한을 적용하지 않습니다.
2026-10-29 사용자가 현재 운용 환경의 손 통신 지연 해결을 확인했습니다.
과거 측정·진단 이력은 [HX5_LATENCY.md](hardware/HX5_LATENCY.md)에 남겨두었습니다.

## Loop와 종료

Loop에서 로봇 소스 `aiworker`와 손 소스 `HX5-D20`를 선택합니다.
관절과 손 압력은 동기화된 두 소스에 기록됩니다. 이전에 등록한 통합 소스나
별도 tactile 소스는 새 설정에서 제외하세요.

맨손 + Loop의 키는 Loop 창에 포커스가 있을 때 동작합니다.

| 키 | 동작 |
|---|---|
| V | 팔·손 시작 / HOLD / 재개 |
| WASD / O·P | mobile 이동 / 회전 |
| 방향키 / U·I | 목 / lift |
| Shift+A 또는 UI | Loop 녹화 시작 |
| C 짧게 / 길게 | Loop 검토에서 실패 저장 / 폐기 |

베이스 이동 후 이동 키를 놓고 V 한 번으로 재앵커·재개합니다.
컨트롤러 입력에서는 A+X로 시작·재개하고 B+Y로 HOLD합니다.
HOLD 중에도 Loop 기록은 계속되며 paused/action_valid를 남깁니다.

PC 터미널 종료는 HOLD 후 로봇 세션을 유지합니다. 로봇 터미널을 종료하면
명령을 중단하고 최소 3초 후 follower를 정리한 다음 손 모터 40개의 토크 OFF를
확인합니다. `[HAND TORQUE OFF] Verified 40/40 finger motors`까지 기다리세요.

## 다른 PC에서 업데이트

양쪽 teleop을 종료한 뒤 기존 clone에서 실행합니다.

```bash
cd ~/Downloads/external_repos/ai_worker_teleop_deploy
git switch main
git pull --ff-only origin main
bash hardware/install_pc.sh
```

이후 [실행 문서 맨 아래](docs/ai_teleop_hand.txt)의 **PC에서 로봇으로 배포**
명령을 실행해야 로봇 컨테이너도 갱신됩니다. 로컬 변경 때문에 pull이 거절되면
해당 변경을 먼저 보존하세요. 새 PC에는 같은 문서의 최초 설치·인증서 생성도
필요합니다. 기존 ROS Jazzy·Cyclo·SH5 모델/드라이버 환경을 사용하며,
Loop 모드에는 로봇의 ffw_loop_streamer·loop_sdk와 PC Loop가 필요합니다.

`teleop.sh pc`와 `docker/container.sh start/enter`는 현재 데스크톱의
`xhost +SI:localuser:root`를 자동 실행합니다. 재부팅 후에도 별도 수동 실행은
필요하지 않습니다. PC 데스크톱에 로그인한 사용자의 터미널에서 실행하세요.
DISPLAY가 없는 SSH 세션에서는 맨손 + Loop 키보드 연결을 시작할 수 없습니다.

## 상세 자료

- [VR·Loop 구조와 기록 채널](hardware/VR_LOOP.md)
- [손 드라이버 상세 기록](hardware/HAND_TRACE.md)
- [HX5 지연 조사와 실패한 우회 실험](hardware/HX5_LATENCY.md)
- [기존 SG2/backport 안내](docs/LEGACY_MAIN_README.md)

별도 빌드는 원본 Cyclo/underlay를 덮어쓰지 않습니다. 진단용 직접 통신 우회는
혼합 통신 시험에서 실패하여 기본 실행에서 차단되어 있습니다. 원본 ROS 드라이버의
손 목표 쓰기를 30Hz로 제한한 별도 빌드를 사용하며 팔 쓰기·피드백 읽기는 유지합니다.
