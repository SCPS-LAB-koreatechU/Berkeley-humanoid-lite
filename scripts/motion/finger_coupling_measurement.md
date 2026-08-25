# 손가락 커플링 실측 절차 (hand.coupled 캘리브레이션)

> 2026-08-25 작성. `ros2_ws/TODO.md` B절과 `scripts/motion/fit_finger_coupling.py`,
> `track_finger_joints.py` 독스트링을 실행 가능한 체크리스트로 정리한 문서.
> 목표: `config/arm_attachment.yaml`의 `hand.coupled` 플레이스홀더
> (`Flexor←Pitch = 1.0`, `DIP←Flexor = 1.0`)를 실측값으로 교체.

## 왜 하는가

손끝 도달거리, 엄지 대향(핀치) 가능 여부, 리타겟팅된 그립의 실제 동작이
전부 이 두 비율에 걸려 있다. 지금 값은 "손가락이 길이에 비례해 말린다"는
모양만 맞는 추정치다. 참고로 WIRobotics ALLEX의 동급 손가락 실측값은
DIP←PIP 0.656, 엄지 IP←MCP 0.732 — 1.0과는 거리가 멀다.

## 준비물

- DexHand 손가락 모듈 (또는 조립된 손) + 구동용 아두이노 (COM15)
- 스마트폰/카메라 + **삼각대 또는 고정 지그** (핸드헬드 절대 금지)
- 손가락 모듈 고정용 클램프
- 조명 (베어링 링이 또렷하게 보이도록)

## 절차

### 0. 커플링 존재 확인 (5분)

손을 고정하고 너클(Pitch) 서보 하나만 0→최대 전 구간 왕복.

- 중절·말절이 근위 마디에 대해 **상대 회전하면** → 커플링 있음, 계속 진행
- 전혀 안 돌면 → 이 빌드는 강체 손가락. `hand.coupled`의 두 multiplier를
  `0.0`으로 기입하고 종료. (이것도 어엿한 실측 결과다)

### 1. 카메라 배치

- 카메라 광축을 손바닥 **y축 방향**(손가락 굴곡축과 평행)으로: 손 옆에서
  촬영해 굴곡이 이미지 평면 안에서 일어나게 한다.
- 손가락 세 굴곡축이 화면에 점으로 보이는 각도가 정답.
- 모듈은 클램프로 고정. 촬영 중 모듈의 **방향**(orientation)만 유지되면
  병진 이동은 허용된다 (`--base` 없이 돌릴 때).
- 조준 오류 경고가 나오면 **재촬영** — 사후 보정 불가.

### 2. 9포즈 스윕 촬영

너클 명령을 0 → 0.95 rad(서보 리밋) 균등 9단계로, 각 포즈에서 1–2초 정지.
영상 하나로 촬영 후 프레임 추출:

```bash
ffmpeg -i sweep.mp4 -vf fps=1 frames/frame_%03d.png
```

(포즈당 정지 사진 9장을 찍어도 무방)

### 3. 관절 각도 추출

베어링 자동 추적 (기본, Middle 손가락):

```bash
python3 scripts/motion/track_finger_joints.py frames/ --finger Middle -o angles.csv
```

- 각 관절의 베어링(흰 플라스틱 위 어두운 링)을 Hough 원 검출로 추적한다.
- 자동 검출이 불안하면 **관절 중심 클릭 모드**가 가장 정밀
  (±2 px → Flexor ±0.013, DIP ±0.022).

### 4. 비율 피팅

```bash
python3 scripts/motion/fit_finger_coupling.py --from-points angles.csv
```

기대 정밀도(9포즈 기준, 1σ):

| 방법 | Flexor | DIP |
|---|---|---|
| 관절 중심 클릭 ±2px | ±0.013 | ±0.022 |
| 각도기 판독 ±2° | ±0.021 | ±0.030 |
| 손끝 마커 2.0mm | ±0.061 | ±0.273 |

(손끝 위치가 아니라 **관절 각도**를 재는 이유: 18mm 레버를 거치며 말절
정밀도가 희석되기 때문)

### 5. 결과 기입 및 재생성

`config/arm_attachment.yaml`:

```yaml
hand:
  coupled:
    Flexor: {driver: Pitch, multiplier: <실측값>}
    DIP: {driver: Flexor, multiplier: <실측값>}
```

손가락마다 값이 다르면 손가락별 분리가 필요한지 판단 후:

```bash
python3 generate_urdf.py
python3 scripts/motion/analyze_hand.py   # 도달거리 표 갱신
```

### 6. 같이 잴 것 (같은 촬영 세션에서)

- [ ] **엄지 어댑터 오프셋**: 인쇄한 어댑터의 실제 오프셋을 재서
      `hand.digit_adapter.Thumb`에 기입 (V2 엄지는 V1 마운트 대비 15mm 어긋남)
- [ ] **엄지 고정 자세**: 조립 각도를 `hand.thumb`에 기입
      (`analyze_hand.py`가 제안하는 핀치 가능 자세 참고)
- [ ] **서보 실제 리밋**: `hand.actuated`(Pitch 0~0.95, Yaw ±0.30)를
      8서보 빌드 실측 리밋과 대조
- [ ] **전완 조립체 무게**: 저울로 재서 관성 스케일 검증
      (현재 PLA 1240 kg/m³ 가정)

## 기록

측정 완료 후 결과(비율, 사용 방법, 오차 추정)를 노션 PM 페이지 SIM 섹션에
날짜와 함께 기록할 것.
