# usd/ — 조립 로봇 USD

`scripts/blender_export_usd.py`가 Blender 조립 씬(blender/bhl_v1arm_assembly.blend)에서 생성.
좌표계 Z-up, 단위 m. 프림 루트 `/bhl`, 링크는 `/bhl/<link>`, 조인트는 `/bhl/joints/<joint>`.

| 파일 | 내용 |
|---|---|
| `bhl_v1arm_visual.usdc` | 시각 전용 — 링크별 Xform + 메시 + 머티리얼 |
| `bhl_v1arm_physics.usdc` | + UsdPhysics: ArticulationRoot, RigidBody/Mass 81, convexHull 콜리전 72, Revolute 48(리밋 °) / Fixed 33, angular Drive |
| `bhl_v1arm_joints.json` | URDF 조인트 테이블 (내보내기 중간 산출물) |

## Isaac Sim/Lab에서 쓸 때

- 표준 UsdPhysics 스키마만 사용 — Isaac이 열 때 PhysX 속성으로 변환됨
- Drive는 stiffness 0 / damping 0.05로 저작됨 → env cfg(ActuatorCfg)에서 관절별로 설정할 것
- **mimic 커플링 없음**: URDF의 Flexor←Pitch, DIP←Flexor(×1.0, 미실측)는 독립 조인트로 들어가 있음. 커플링은 (a) 컨트롤러에서 강제, (b) Isaac의 mimic joint API로 후처리, (c) 실측 후 재생성 중 택일
- 콜리전은 시각 메시의 convexHull 근사 — 정밀 접촉이 필요하면 손가락만 SDF/분해 콜리전으로 교체
- 공식 파이프라인(Isaac URDF importer, `source/berkeley_humanoid_lite_assets/scripts/convert_urdf_to_usd.py`)과 병행 가능 — 이 USD는 Blender 왕복 검증·시각화·빠른 반복용

## 재생성

Blender에서 조립 씬 로드(또는 `blender_import_urdf.py` 실행) 후:

```python
exec(open(r"...\scripts\blender_export_usd.py", encoding="utf-8").read())
usd_stage1_export(); usd_stage2_physics()
```
