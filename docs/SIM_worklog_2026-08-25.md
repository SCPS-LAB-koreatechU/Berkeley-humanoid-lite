# SIM 작업 기록 — 2026-08-25

> 작성: Claude (Cowork 세션, 정찬우 요청). URDF/USD 정비 + 휴머노이드↔DexHand 결합부 + SIM to REAL 준비 작업.

## 1. 자산 복구 (누락 메시)

- `source/berkeley_humanoid_lite_assets` 서브모듈이 빈 상태 → 핀 커밋 **fc90fed** 기준으로 클론해 복구
  - 몸통/팔/다리 시각 메시 STL **26개** + 상류 URDF 3종(`berkeley_humanoid_lite.urdf`, `_biped.urdf`, `config.json`) 커밋
  - 대형 USD 2종(base 60MB / biped 34MB)은 전송 한도(20MB/파일) 초과로 미전송 — 필요 시 GitHub에서 직접 수령
- Windows 체크아웃의 git 심링크 3종(`meshes`, `meshes_v1`, `meshes_dexhand`)이 텍스트 파일로 깨진 것 확인 → Blender 쪽에서 경로 직접 매핑으로 우회

## 2. 결합부(mount) 실측·설계 — TODO A절 해소

리포 메시에서 직접 측정한 값:

| 항목 | 측정값 |
|---|---|
| elbow_roll 출력 허브 | Ø10.0 × 10mm |
| 스톡 핸드 볼트 패턴 | 6×Ø3.4(M3) @ BCD Ø36.0, 각도 30°+60k |
| V1 전완 근위 커프 | 환형 OD 84.0 / ID 70.0mm |

**어댑터 설계** (`meshes_adapter/bhl_v1_forearm_adapter.stl`, mm 단위, 3D 프린팅용):
- 스택: 베이스판 8mm(스톡 볼트패턴 + Ø10.6 허브 보어 + M3 카운터보어) → 엔드스톱 플로어 4mm(Ø84) → 삽입 칼라 12mm(OD 69.2, 벽 5mm, 반경 여유 0.4mm)
- 측면 M3 셀프탭 파일럿 Ø2.6 ×4 (45°+90k, 셸 분할선 회피)
- 좌우 공용(xz 대칭 설계), 부피 41cm³, 질량 가정 50.8g(솔리드 PLA 1240kg/m³)
- ⇒ **mount.right = xyz [0, 0, 0.012], rpy [0, 0, 0]** 확정 (셸 안착면 = 플랜지에서 12mm)

## 3. URDF 반영 및 재생성

- `config/arm_attachment.yaml`: mount 값 + `mount.adapter` 블록(메시/질량/COM/관성) 기입, "STILL A GUESS" 주석을 설계 근거로 교체
- `generate_urdf.py`: `attach_adapter()` 추가 — mount 링크에 어댑터 visual/collision/inertial 부착(역변환 원점: mount 재튜닝 시 어댑터는 플랜지에 고정), 튜닝 모델에는 정렬용 visual
- 사용자 PC에서 URDF 4종 재생성 완료 (v1arm: 82링크/81관절/48구동)

## 4. 간섭 검토 (클라우드 기하 검증)

- 어댑터 ↔ elbow_pitch 하우징: 0.09mm (스톡과 동일한 볼트 결합면 — 정상)
- 전완 커프 ↔ 하우징: 14mm 여유
- 손목 3축 ROM 스윕 최소 여유: 0.47mm (wrist_yaw 최대에서 V1 자체 설계 여유)

## 5. Blender MCP 환경 구축 + 로봇 조립

- 원인 규명: 기존 "Blender MCP v1.5"(ahujasid 계열) 애드온이 포트 9876 점유 + 프로토콜 불일치 → 타임아웃
- 조치: 구버전 비활성화 → **공식 Blender MCP 확장**(`bl_ext.user_default.mcp`, mcp-1.0.0) 설치·활성화, `use_online_access=True`, 환경설정 저장(재시작 후 자동 시작)
- `scripts/blender_import_urdf.py` 작성 — URDF를 라이브 Empty 트리로 조립(82링크/72메시, 0.7초), `bhl_set_joint()`/`bhl_zero_pose()`로 관절 포즈 가능
- 손목 굽힘 + 손가락 컬링 포즈 테스트 통과, 결과 씬 `blender/bhl_v1arm_assembly.blend` 저장, 렌더 `blender/bhl_v1arm_assembly_view.png`

## 6. 손가락 굽힘 실측 준비 — TODO B절

- 실측 절차 체크리스트 작성: `scripts/motion/finger_coupling_measurement.md`
  (커플링 존재 확인 → y축 방향 고정 촬영 → 9포즈 스윕 → `track_finger_joints.py` → `fit_finger_coupling.py` → yaml 기입)
- 스윕 촬영본은 아직 없음 — 촬영 후 처리 예정

## 7. 어댑터 출력 패키지 (오후 추가 작업)

- cadquery 파라메트릭 재모델링 → **STEP** 출력 (`cad/bhl_v1_forearm_adapter.step`, 체적 40.97cm³로 STL과 일치 검증)
- 치수 도면 PDF(`cad/bhl_v1_forearm_adapter_drawing.pdf`) + 출력·조립 가이드(`cad/adapter_print_guide.md`, BOM: 6×M3×10 SHCS + 4×M3×8 셀프탭)
- 공차: 허브 반경 0.3 / 칼라 반경 0.4mm — 첫 출력 후 확인 절차 포함

## 8. USD 제작 (visual + physics)

- `scripts/blender_export_usd.py` 작성 — 2단계 파이프라인:
  1. 링크 트리를 플랫 구조로 펼쳐 **시각 USD** 내보내기 (`usd/bhl_v1arm_visual.usdc`, 50.9MB, Z-up·m 단위)
  2. Blender 내장 pxr로 **UsdPhysics 저작** (`usd/bhl_v1arm_physics.usdc`): ArticulationRoot(/bhl) + RigidBody·Mass(URDF 관성 → 주축 분해) 81개 + convexHull 콜리전 72개 + RevoluteJoint 48(리밋 °변환, 축 정렬 쿼터니언) + FixedJoint 33 + angular Drive(강성 0/감쇠 0.05 — Isaac env에서 튜닝)
- usd-core 검증: 조인트 참조 무결, R_Index_Pitch 축이 URDF 축과 일치(0.08754, 0.99616, −0.00043), 총 질량 10.085kg, base COM z=0.675 ✓
- 한계: URDF `<mimic>`(Flexor←Pitch, DIP←Flexor)은 UsdPhysics에 없음 → 독립 구동 조인트로 내보냄, 커플링은 컨트롤러에서 강제하거나 Isaac 쪽 mimic API로 후처리

## 다음 할 일

- [ ] 어댑터 3D 프린팅 → 실물 조립 후 무게 실측(관성 보정), 튜닝 모델로 mount 재검증
- [ ] 손가락 커플링 스윕 촬영 → 비율 피팅 → `hand.coupled` 교체
- [ ] 엄지 digit_adapter 15mm 오프셋 실측, 엄지 고정 자세 기입
- [ ] (Isaac용) 보정된 URDF → USD 변환 파이프라인
