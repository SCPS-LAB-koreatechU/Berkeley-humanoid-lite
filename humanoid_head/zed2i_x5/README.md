# 🎥 Humanoid Head: ZED 2i on RDK X5 (SDK 없이)

BHL 머리에 장착하는 **ZED 2i** 스테레오 카메라를 **D-Robotics RDK X5**에서 ZED SDK 없이 ROS 2 토픽으로 퍼블리시하는 설정입니다.
ZED SDK는 CUDA(NVIDIA GPU)가 필요해서 X5에서는 실행할 수 없기 때문에, 영상과 IMU를 아래처럼 나눠서 처리합니다.

| 데이터 | 노드 | 출처 |
|---|---|---|
| 스테레오 영상 | `hobot_zed_cam` (`anypub_stereo_imgs`) | D-Robotics TROS 기본 패키지 |
| IMU / 자력계 / 기압 / 온도 / 습도 | `zed_imu_node` (이 폴더) | [zed-open-capture](https://github.com/stereolabs/zed-open-capture) 기반 C++ 노드 |

---

## 📂 구성

```
humanoid_head/zed2i_x5/
├── zed_imu_ros2/              # ROS 2 패키지 (ament_cmake)
│   ├── src/zed_imu_node.cpp   # IMU + 보조 센서 퍼블리셔
│   ├── src/zed_reset.cpp      # 카메라 MCU 리셋 (영상 링크 복구용)
│   ├── launch/zed_imu.launch.py   # IMU만
│   ├── launch/zed_all.launch.py   # 리셋 → 영상 → IMU (권장)
│   └── config/zed_imu.yaml
├── udev/99-slabs.rules        # ZED USB 권한 규칙
└── tools/measure_rates.py     # 토픽 주기 측정 도구
```

## 🧰 테스트 환경

- RDK X5 (arm64, Ubuntu 22.04), ROS 2 Humble + TROS Humble (`tros-humble-hobot-zed-cam` 2.3.3)
- ZED 2i (펌웨어 3.9), zed-open-capture `9373915`
- 카메라는 USB3 액티브 연장 허브 2개를 거쳐 연결됨

## 🚀 설치

### 1. 의존성 + udev 규칙
```bash
sudo apt install libhidapi-dev libusb-1.0-0-dev libopencv-dev
sudo cp humanoid_head/zed2i_x5/udev/99-slabs.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 카메라를 다시 꽂기
```

### 2. zed-open-capture 빌드
```bash
mkdir -p ~/bhl/third_party && cd ~/bhl/third_party
git clone https://github.com/stereolabs/zed-open-capture.git
cd zed-open-capture && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
```
> 라이브러리는 시스템에 설치하지 않고 빌드 폴더(`build/libzed_open_capture.so`)를 직접 링크합니다.
> 다른 경로에 받았다면 아래 colcon 빌드에서 `-DZED_OC_DIR=<경로>`를 추가하세요.

### 3. ROS 2 패키지 빌드
```bash
mkdir -p ~/bhl/zed_ws/src
ln -s <이 저장소 경로>/humanoid_head/zed2i_x5/zed_imu_ros2 ~/bhl/zed_ws/src/
cd ~/bhl/zed_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## ▶️ 실행

```bash
source /opt/tros/humble/setup.bash
source ~/bhl/zed_ws/install/setup.bash

ros2 launch zed_imu_ros2 zed_all.launch.py        # 영상 + IMU (권장)
ros2 launch zed_imu_ros2 zed_imu.launch.py        # IMU만
```

`zed_all.launch.py` 인자:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `resolution` | `720p` | 영상 해상도 |
| `pub_bgr` | `false` | `true`면 영상을 `bgr8`로 퍼블리시 (rqt에서 보기용, 약 4.4 Hz로 느려짐) |
| `imu_delay` | `5.0` | 영상 노드 시작 후 IMU 노드를 띄우기까지 대기 시간(초) |
| `reset` | `true` | 시작 전에 카메라 MCU 리셋 (아래 트러블슈팅 참고) |

`config/zed_imu.yaml` 파라미터:

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `serial_number` | `-1` | `-1`이면 처음 찾은 ZED 사용 |
| `frame_id` | `zed_imu_link` | 메시지 frame |
| `publish_aux` | `true` | 자력계/기압/온도/습도 퍼블리시 |
| `reliable_qos` | `true` | reliable QoS (rqt 등 reliable 구독자와 호환). `false`면 best effort |
| `gyro_bias_calib_sec` | `0.0` | `>0`이면 시작할 때 N초 동안 자이로 바이어스를 측정해서 뺌 (**카메라 정지 상태 필수**) |

## 📡 토픽 및 실측 주기

`tools/measure_rates.py`로 측정했습니다(영상과 IMU 동시 실행, 2026-10-12).

| 토픽 | 타입 | 실측 |
|---|---|---|
| `/image_combine_raw` | `sensor_msgs/Image` (NV12, 1280×1440, 좌/우를 위아래로 합침) | 약 9.5 Hz |
| `/zed/imu/data_raw` | `sensor_msgs/Imu` (m/s², rad/s, orientation 없음) | 약 370–390 Hz |
| `/zed/imu/mag` | `sensor_msgs/MagneticField` (T) | 50 Hz |
| `/zed/atm_press` | `sensor_msgs/FluidPressure` (Pa) | 25 Hz |
| `/zed/humidity` | `sensor_msgs/RelativeHumidity` (0–1) | 25 Hz |
| `/zed/temperature/imu`, `/left`, `/right` | `sensor_msgs/Temperature` (°C) | 25 Hz |

- CPU 사용률: `anypub_stereo_imgs` 약 65%, `zed_imu_node` 약 18%
- 같은 카메라를 UVC로 직접 캡처하면 30/60/100 fps가 나옵니다(`v4l2-ctl`로 측정). ROS 영상이 약 9.5 Hz인 것은 `hobot_zed_cam` 노드 안의 변환 처리 때문입니다.
- 정지 상태 IMU 노이즈: 가속도 표준편차 약 0.009 m/s², 자이로 약 0.08 °/s, 자이로 Y축 바이어스 약 0.3 °/s

## 🔧 트러블슈팅 / 알려진 이슈

- **rqt Image View에서 `Unrecognized image encoding [nv12]`**: 기본 영상은 NV12라서 rqt/cv_bridge가 표시하지 못합니다. 영상을 눈으로 확인할 때는 다음처럼 실행하세요(대신 약 4.4 Hz로 느려집니다).
  ```bash
  ros2 launch zed_imu_ros2 zed_all.launch.py pub_bgr:=true
  ```
- **rqt에서 `offering incompatible QoS ... RELIABILITY`**: best effort 퍼블리셔는 reliable 구독자(rqt 등)와 연결되지 않습니다. 그래서 `zed_imu_node`는 기본값이 reliable입니다(`reliable_qos: true`).

- **`serial_num error`로 영상 노드가 죽음, 또는 프레임이 안 들어옴**
  `hobot_zed_cam`이 종료될 때 비정상 종료(abort)하면서 USB3 영상 링크가 멈추는 경우가 있습니다(`dmesg`에 `status (-71)`). 이후 실행은 계속 실패합니다.
  → `zed_all.launch.py`는 시작 전에 `zed_reset`(MCU 전체 리셋)을 자동으로 실행합니다. 수동으로 리셋하려면:
  ```bash
  ros2 run zed_imu_ros2 zed_reset
  ```
- **영상 노드와 IMU 노드를 동시에 시작하면 안 됨**: 둘이 같은 HID 인터페이스를 동시에 열면 영상 노드가 실패합니다. 그래서 `imu_delay`(5초)를 둡니다.
- **카메라 자체 타임스탬프는 사용하지 않음**: 호스트보다 약 130초 늦고 초당 약 7 ms씩 어긋나서, 수신 시점의 ROS 시간을 사용합니다(평균 지연 약 5 ms).
- **자력계는 보정되지 않은 원시값**입니다(정지 상태 크기 약 1.8 µT). Yaw 추정에 쓰려면 별도 보정이 필요합니다.
- **zed-open-capture 빌드 옵션 주의**: 라이브러리가 `VIDEO_MOD_AVAILABLE`로 빌드되면 `SensorCapture` 클래스 구조가 달라집니다. 이 패키지는 같은 define으로 빌드합니다(`CMakeLists.txt`). 다르게 빌드하면 힙 손상으로 크래시가 납니다.
- `hobot_zed_cam`은 Ctrl+C로 종료할 때 `RCLError`로 abort합니다(벤더 패키지 문제, 동작에는 영향 없음).
- 오리엔테이션이 필요하면 `imu_filter_madgwick` 같은 필터를 `/zed/imu/data_raw` 뒤에 붙이세요.
