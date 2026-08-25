#!/usr/bin/env python3
"""Parametric CAD (cadquery) rebuild of the forearm adapter -> STEP + STL."""
import math
import cadquery as cq

# ---- parameters (mm) — keep in sync with make_adapter.py / yaml notes ----
PLATE_D, PLATE_H = 44.0, 8.0
FLOOR_D, FLOOR_H = 84.0, 4.0
COLLAR_OD, COLLAR_ID, COLLAR_H = 69.2, 59.2, 12.0
HUB_BORE_D, HUB_BORE_DEPTH = 10.6, 11.0
BCD, HOLE_D, CBORE_D, CBORE_FROM_Z = 36.0, 3.4, 6.5, 4.0
PILOT_D, PILOT_Z, PILOT_ANGLES = 2.6, 18.0, (45, 135, 225, 315)

z_floor_top = PLATE_H + FLOOR_H            # 12
z_top = z_floor_top + COLLAR_H             # 24

solid = (cq.Workplane("XY")
         .circle(PLATE_D / 2).extrude(PLATE_H)
         .faces(">Z").workplane()
         .circle(FLOOR_D / 2).extrude(FLOOR_H)
         .faces(">Z").workplane()
         .circle(COLLAR_OD / 2).extrude(COLLAR_H))

# collar bore (leaves an 11 mm-deep hub pocket floor below)
solid = solid.cut(cq.Workplane("XY").workplane(offset=HUB_BORE_DEPTH)
                  .circle(COLLAR_ID / 2).extrude(z_top - HUB_BORE_DEPTH + 1))
# hub bore
solid = solid.cut(cq.Workplane("XY").workplane(offset=-1)
                  .circle(HUB_BORE_D / 2).extrude(HUB_BORE_DEPTH + 1))
# 6 x M3 clearance + counterbores on the bolt circle
for k in range(6):
    a = math.radians(30 + 60 * k)
    x, y = BCD / 2 * math.cos(a), BCD / 2 * math.sin(a)
    solid = solid.cut(cq.Workplane("XY").workplane(offset=-1).center(x, y)
                      .circle(HOLE_D / 2).extrude(z_floor_top + 2))
    solid = solid.cut(cq.Workplane("XY").workplane(offset=CBORE_FROM_Z)
                      .center(x, y).circle(CBORE_D / 2)
                      .extrude(z_floor_top - CBORE_FROM_Z + 1))
# 4 radial self-tap pilots through the collar wall
for ang in PILOT_ANGLES:
    pil = (cq.Workplane("YZ").workplane(offset=COLLAR_ID / 2 - 2)
           .center(0, PILOT_Z).circle(PILOT_D / 2)
           .extrude(COLLAR_OD / 2 - COLLAR_ID / 2 + 6))
    solid = solid.cut(pil.rotate((0, 0, 0), (0, 0, 1), ang))

vol = solid.val().Volume() / 1000.0
print(f"volume: {vol:.1f} cm3 (trimesh reference: 41.0)")
cq.exporters.export(solid, "/home/claude/work/bhl_v1_forearm_adapter.step")
cq.exporters.export(solid, "/home/claude/work/bhl_v1_forearm_adapter_cq_mm.stl",
                    tolerance=0.01, angularTolerance=0.1)
print("exported STEP + STL")
