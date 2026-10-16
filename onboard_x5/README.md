# bhl_onboard_ws — Berkeley Humanoid Lite 보드 내 제어단 (RDK X5)

> 이 폴더는 colcon 워크스페이스 그대로입니다. 보드에는 `~/bhl_onboard_ws`로 배포됩니다(`./deploy_to_x5.sh`).
> PC 쪽 짝: [Berkeley-Humanoid-Lite-origin- `onboard-x5` 브랜치](https://github.com/SCPS-LAB-koreatechU/Berkeley-Humanoid-Lite-origin-/tree/onboard-x5)
> — `demo.launch.py onboard:=true`, `moveit_controllers_onboard.yaml`.

외부 PC가 IK/MoveIt 등으로 계산한 결과를 토픽으로 보내면, 로봇에 실린 RDK X5가 받아서
recoil 모터(CAN)를 구동하는 **보드 측 저수준 제어 패키지**입니다.

- 메시지·토픽 구조는 **unitree_sdk2**(`rt/lowcmd` / `rt/lowstate`, `MotorCmd{q,dq,tau,kp,kd}`, CRC)를 따랐습니다.
- 모터 프로토콜·관절 규약은 **berkeley-humanoid-lite** lowlevel(recoil, `robot/humanoid.py`·`bimanual.py`)과
  이 PC의 `hardware_bridge.py`(벤치에서 검증된 enable 순서·버스 함정 대응)를 따랐습니다.

```
[외부 PC]  MoveIt / IK / 리타겟팅
   │  ROS 2 DDS, ROS_DOMAIN_ID=77 (Wi-Fi/유선)
   ▼
[RDK X5]
 bhl_commander  ── FollowJointTrajectory 액션 ─ 궤적을 보드에서 200 Hz로 샘플링 (Wi-Fi 지터 차단)
   │               손 프리셋 서비스
   │  /lowcmd (LowCmd, CRC)          ← 또는 PC가 직접 /lowcmd 스트리밍 (unitree 저수준 방식)
   ▼
 bhl_lowlevel   ── 200 Hz 제어 루프, 상태 머신 IDLE / DAMPING / ACTIVE / ESTOP
   │               안전층: 관절 범위 클램프, 속도(슬루) 제한, 게인 상한, 명령 끊김 → hold → damping
   │               /lowstate, /bhl/joint_states, /bhl/board_state
   ▼
 can0: 왼팔 recoil 1·3·5·7·9  (+ 왼손·손목 ESP32, 17모터 — 프로토콜 미정)
 can1: 오른팔 recoil 2·4·6·8·10 (+ 오른손·손목 ESP32)
```

## 토픽 / 서비스

| 이름 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/lowcmd` | `bhl_msgs/LowCmd` | → 보드 | 22슬롯 고정 배열 (`mode,q,dq,tau,kp,kd`), CRC 필수. **발행자는 한 명만** |
| `/lowstate` | `bhl_msgs/LowState` | 보드 → | 100 Hz, 슬롯별 `mode,q,dq,error,lost` + board_state |
| `/hand/{left,right}/cmd` · `/state` | `HandCmd` / `HandState` | ↔ | 손·손목 17모터, 50 Hz |
| `/bhl/joint_states` | `sensor_msgs/JointState` | 보드 → | 보드 모터만. PC에서 전체 URDF `/joint_states`로 병합 |
| `/bhl/board_state` | `bhl_msgs/BoardState` | 보드 → | 2 Hz. 상태, 루프 Hz, 버스 상태, 고장 목록 |
| `/arms_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 액션 | MoveIt이 그대로 사용 (이름이 `moveit_controllers_hardware.yaml`과 동일) |
| `/bhl_lowlevel/enable` | `std_srvs/SetBool` | 서비스 | true: 측정 자세에서 ACTIVE, false: IDLE |
| `/bhl_lowlevel/{damping,estop,reset_estop,zero}` | `std_srvs/Trigger` | 서비스 | `zero`는 IDLE에서만, 현재 자세를 0으로 저장 |
| `/bhl/hand_preset` | `bhl_msgs/HandPreset` | 서비스 | `side`, `name`, `duration` |

슬롯 표(LowCmd 인덱스): 0–4 왼팔(CAN 1,3,5,7,9), 5–9 오른팔(CAN 2,4,6,8,10), 10–21 다리(예약).
정확한 표는 `src/bhl_onboard/config/bhl_x5.yaml`의 `motors`입니다.

`MotorCmd.kp/kd/tau`는 빠른 위치 프레임이 아니라 **펌웨어 파라미터(SDO)**로 들어갑니다 — recoil PDO_2에는
위치·속도만 실립니다. 바뀔 때만, 모터당 0.2 s에 한 번 이하로 쓰고, `safety.caps`로 상한을 둡니다. 0이면 설정값 유지.

## 실행

```bash
# 이 PC에서 (실물 없이) — 빌드 + 단위 18개 + 종단간 23개
./test.sh
tools/bench_sim.sh            # 루프 Hz / CPU / 최악 지연
tools/moveit_onboard_sim.sh   # PC MoveIt(onboard:=true) + 보드 시뮬 전체 경로

# 보드에서
./run_onboard.sh              # 실물 (IDLE로 시작 — 아무것도 안 움직임)
./run_onboard.sh sim:=true    # 가짜 CAN
./run_onboard.sh commander:=false   # PC가 /lowcmd 직접 구동할 때
```

### X5에 배포 (보드가 네트워크에 연결되면)

```bash
# PC에서. 처음 한 번은 --setup (apt 패키지, CAN 자동 bring-up udev 규칙, 빌드, 단위 테스트)
./deploy_to_x5.sh <user>@<x5-ip> --setup
# 이후 코드 수정 시
./deploy_to_x5.sh <user>@<x5-ip>
# 보드에서: 이 보드가 200 Hz를 감당하는지 먼저 확인
ssh <user>@<x5-ip> 'cd ~/bhl_onboard_ws && tools/bench_sim.sh'
```

### PC에서 MoveIt으로 보드 구동

```bash
cd ~/Berkeley-Humanoid-Lite/ros2_ws
./run_demo.sh onboard:=true          # PC 브리지를 띄우지 않고 보드의 arms_controller로 실행
ros2 service call /bhl_lowlevel/enable std_srvs/srv/SetBool '{data: true}'
ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py left_arm ready
```

`onboard:=true`는 `hardware:=true`(PC가 직접 CAN 구동)와 동시에 쓸 수 없게 막아 두었습니다.

### PC에서 /lowcmd 직접 (unitree g1_dual_arm_example 대응)

```bash
# 보드: ./run_onboard.sh commander:=false
ros2 run bhl_onboard bhl_lowcmd_example --joint arm_left_shoulder_yaw_joint --amplitude 0.2
```

C++ 등 다른 클라이언트는 `bhl_onboard/crc.py`의 패킹(`<B` + 22×`<B5fI`, zlib CRC-32)으로 `crc`를 채우면 됩니다.

## 안전 동작 (전부 시뮬 테스트로 확인)

| 상황 | 보드 동작 |
|---|---|
| enable | 각 모터 **현재 측정 위치**에서 시작(점프 없음). 측정값이 URDF 범위 ±3° 밖이면 그 모터 거부(offset 미설정 의심) |
| `/lowcmd` 0.25 s 끊김 | 마지막 목표 유지(hold) |
| 추가 1.0 s 끊김 | 펌웨어 DAMPING (팔이 천천히 내려옴) |
| enable 이전에 받은 명령 | 무시 |
| `/lowcmd` 발행자 2개 이상 | 전부 무시 + 고장 표시 (commander와 PC 직접 구동이 섞이는 것 방지) |
| CRC 불일치 | 버림, `rejected_crc` 카운트 |
| 목표가 범위 밖 | 한계−0.05 rad로 클램프. 궤적 포인트가 URDF 범위 밖이면 commander가 goal 거부 |
| 목표 급변 | 1 rad/s 슬루 제한 |
| 모터 응답 100사이클 없음 | 그 모터만 IDLE + LOST, 나머지 유지 |
| 버스 무응답(모터 전원 꺼짐) | 초당 ping 1개만 보냄, enable 시 프레임 0개 (gs_usb가 BUS-OFF에서 스스로 못 돌아오는 문제 예방) |
| ENOBUFS(송신 큐 막힘) | 0.5 s 송신 보류 |
| 어댑터 뽑힘/재꽂힘 | 0.5 s 주기로 감지해 닫고 다시 엶. 구동 중이던 모터는 BUS 고장 → 다시 enable 필요 |
| E-STOP | DAMPING 래치, `reset_estop` 후 enable |
| 노드 종료(SIGINT/SIGTERM) | 구동 모터 전부 IDLE 후 종료 |
| `enabled: false` 모터 (현재 CAN 8) | 읽기만, 절대 구동 안 함 |

## 실물 bring-up 순서 (권장)

1. 모터 전원 OFF로 `./run_onboard.sh` → `/bhl/board_state`에 `nothing answers` 확인(정상).
2. 모터 전원 ON → `motor N answering` 로그, `ros2 topic echo /bhl/joint_states`.
3. 손으로 관절을 돌려 **부호(direction)** 확인. URDF 0 자세에서 `ros2 service call /bhl_lowlevel/zero std_srvs/srv/Trigger`
   (offset이 `~/.config/bhl_onboard/offsets.yaml`에 저장되고 yaml 값을 덮어씀).
4. `bhl_x5.yaml`에서 처음엔 관절 1개만 `enabled: true` → enable → `bhl_lowcmd_example --amplitude 0.05`.
5. 하나씩 늘린 다음 MoveIt `onboard:=true`.

**모터 전원을 끄기 전에 보드 노드를 먼저 끄세요** (무응답 버스로 프레임이 계속 나가면 어댑터가 굳는 문제, 메모리 기록 참고).

## RDK X5 실측 (2026-10-16, 192.168.1.225)

- 보드: RDK X5 V1.0, Cortex-A55 ×8 @1.5 GHz, Ubuntu 22.04.5, 커널 6.1.83, `/opt/ros/humble` + `/opt/tros/humble`,
  python-can 4.6.1, control_msgs·colcon·pytest 설치됨 → `setup_x5.sh`의 apt 단계 불필요. 빌드 1분 27초.
- **내장 can0(TCAN4550, SPI 18 MHz)은 팔 제어에 못 씀**: 내부 루프백 실측 프레임당 왕복 ~1.1 ms, ~900 프레임/s,
  10모터 한 사이클 9.6 ms. → **팔마다 USB-CAN(gs_usb) 어댑터**. 그때 이름은 can1, can2…(udev로 고정 권장).
- **rclpy executor가 깨어날 때마다 CPU ~7 ms** (wait set을 파이썬에서 재구성). 그래서 고주기 토픽(/lowcmd,
  /lowstate, 손)은 executor 밖에서 제어 스레드가 직접 꺼낸다(`polled.py`). 메시지 객체도 재사용.
- `tools/bench_sim.sh`(시뮬, 버스 1개): control 200 / commander 100 Hz → 루프 200 Hz, lowlevel 73%·commander 45% CPU,
  최악 지연 14 ms(가끔 2~3 사이클 누락 — 모터는 마지막 목표 유지). 100/100 Hz → 지연 0 ms.
  보행 정책처럼 지터가 중요한 용도면 lowlevel을 C++로 옮기는 것이 다음 단계.
- 보드 `~/.bashrc`는 `ROS_DOMAIN_ID=10`(ZED 작업용). `env.sh`가 77로 덮어쓴다(`BHL_ROS_DOMAIN_ID`로 변경).
- 이 보드에서 `--symlink-install`은 파이썬 소스를 복사해 둔다 → 코드 수정 후 반드시 재빌드(`deploy_to_x5.sh`가 함).

## 아직 안 된 것 / 확인 필요

- **ESP32 손·손목 프로토콜 미구현** — `hand.py`의 `Esp32Hand`가 자리만 잡혀 있음. CAN ID가 recoil
  ID 공간(`기능<<7 | 장치ID`)과 겹치면 안 됨. 프로토콜이 정해지면 `driver: esp32`로 전환.
  그 전까지 `driver: none`(아무 프레임도 안 보냄). 손 관절 이름(`joints`)도 URDF 이름으로 매핑 필요 —
  MoveIt arms_controller의 손목 6관절은 지금 `unmapped_joints: ignore`로 경고 후 건너뜀.
- **X5에 USB-CAN 어댑터 미연결** — 실물 CAN 경로는 아직 X5에서 한 번도 안 돌림.
- **실물 CAN 미검증**: 일괄 PDO 교환(모든 모터 송신 후 응답 수집)은 원본 lowlevel의 모터별 송수신과
  다른 방식. 응답 대기 3 ms(`reply_budget`). 펌웨어가 PDO_2 연속 수신을 문제없이 처리하는지 실물에서 확인 필요.
- 다리 슬롯(10–21)은 예약만, IMU 드라이버 없음(`imu_state`는 0).
- tau 피드포워드는 SDO로 쓰므로 느림(0.2 s 간격) — 빠른 토크 제어가 필요하면 펌웨어 PDO 확장 필요.

## 파일

```
src/bhl_msgs/           LowCmd/LowState/MotorCmd/MotorState/IMUState/HandCmd/HandState/BoardState/BusState, HandPreset.srv
src/bhl_onboard/
  bhl_onboard/recoil.py         recoil CAN 일괄 드라이버 (ENOBUFS·무응답·에러프레임 처리)
  bhl_onboard/core.py           상태 머신·안전층·버스 교환 (ROS 없음, 테스트 대상)
  bhl_onboard/lowlevel_node.py  bhl_lowlevel 노드
  bhl_onboard/commander_node.py bhl_commander (궤적 액션, 손 프리셋)
  bhl_onboard/hand.py           손 드라이버 (none/sim/esp32-TODO)
  bhl_onboard/fake_can.py       시뮬 recoil 모터
  bhl_onboard/crc.py            LowCmd CRC
  bhl_onboard/lowcmd_example.py unitree 스타일 직접 제어 예제
  config/bhl_x5.yaml            모터 표·안전값 (2026-10-16 벤치 상태에서 복사)
  test/test_core.py, test/e2e_sim.py
env.sh run_onboard.sh test.sh setup_x5.sh deploy_to_x5.sh tools/
```
