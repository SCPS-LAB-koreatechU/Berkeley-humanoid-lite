"""Put the upper body on an ERP-42 class AMR and export STLs.

The AMR replaces the legs as the lower body. It plugs into the same two
hip-roll output flanges the legs bolt to, through a small adapter (two 45 deg
wedges, a plate, four 2020 posts and a deck plate). The torso is placed so
that, with the elbows bent 90 deg (forearms forward), the lowest point of the
hands is 3 cm above a 75 cm worktable.

Only the wheelbase, the wheel radius and the steering limit are public ERP-42
values. Body size, deck height and track are PLACEHOLDERS (marked "assumed"
below and in the json) until the real vehicle is measured; change them with
--set, e.g. --set deck_z=580 body_len=1650.

Frames: vehicle frame, origin on the ground under the wheelbase centre,
x forward, z up, millimetres.

Usage:
    python3 build_amr_stl.py                     # all three STLs into export/
    python3 build_amr_stl.py --set mount_x=380   # torso further forward
    python3 build_amr_stl.py --no-robot          # skip the 60 MB full assembly
"""

import argparse
import json
import os
import xml.etree.ElementTree as ET

import numpy as np

import merge_urdf_stl as m

# value, source
PARAMS = {
    # public ERP-42 values (maker spec and the open ERP-42 ROS packages)
    "wheelbase": (1040.0, "public"),
    "wheel_r": (265.0, "public"),
    "steer_max_deg": (28.0, "public (ROS param; some papers use 20)"),
    # vehicle envelope: placeholders until measured
    "track": (900.0, "assumed"),
    "tire_w": (170.0, "assumed"),
    "body_len": (1800.0, "assumed"),
    "body_w": (1100.0, "assumed"),
    "deck_z": (600.0, "assumed"),          # deck plate bottom; top = deck_z + deck_t
    "deck_t": (20.0, "assumed"),
    "tub_len": (1700.0, "assumed"),
    "tub_w": (660.0, "assumed"),
    "tub_bottom": (220.0, "assumed"),
    # task requirement
    "table_z": (750.0, "requirement: 75 cm worktable"),
    "hand_clear": (30.0, "requirement: hands 3 cm above the table"),
    "mount_x": (0.0, "requirement: torso at the deck centre"),
    # adapter, robot base frame (z from the base origin)
    "adapter_z": (500.0, "design: 13.7 mm under the hip-roll housings"),
    "adapter_x0": (-75.0, "design"),
    "adapter_x1": (15.0, "design"),
    "adapter_w": (250.0, "design"),
    "flange": (64.0, "design: square pad on the hip-roll output flange"),
    "plate_t": (8.0, "design"),
    "post": (20.0, "design: 2020 extrusion"),
    "post_dx": (35.0, "design"),
    "post_dy": (70.0, "design"),
    "base_plate_x": (220.0, "design"),
    "base_plate_y": (280.0, "design"),
}

# working pose: elbows at 90 deg, forearms pointing forward
WORK_POSE = {"arm_right_elbow_pitch_joint": -1.5708, "arm_left_elbow_pitch_joint": 1.5708}
# hip-roll joint = output flange of the hip-roll actuator, base frame, mm
# (upstream berkeley_humanoid_lite.urdf, leg_*_hip_roll_joint); both axes are
# tilted 45 deg forward-up. The housing has a 4 mm lip outboard of the joint.
HIP_J = np.array([-29.0919, 80.0, 542.631])
HIP_AX = np.array([0.70711, 0.0, 0.70711])
HIP_LIP = 4.5


def _hexa(V):
    """Closed mesh from 8 corners V[(i, j, k)] of a convex hexahedron."""
    quads = [[(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)], [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)],
             [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)],
             [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)], [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]]
    c = np.mean(list(V.values()), axis=0)
    tris = []
    for q in quads:
        a, b, d, e = (np.asarray(V[k], float) for k in q)
        for t in ([a, b, d], [a, d, e]):
            if np.dot(np.cross(t[1] - t[0], t[2] - t[0]), t[0] - c) < 0:
                t = [t[0], t[2], t[1]]
            tris.append(t)
    return np.array(tris)


def box(cx, cy, cz, sx, sy, sz):
    return _hexa({(i, j, k): [cx + (i - 0.5) * sx, cy + (j - 0.5) * sy, cz + (k - 0.5) * sz]
                  for i in (0, 1) for j in (0, 1) for k in (0, 1)})


def wheel(cx, cy, cz, r, w, steer=0.0, n=48, dy=0.0):
    """Cylinder about y (a tyre), steered about z; dy shifts it along its own axle."""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(t), np.full(n, dy), r * np.sin(t)], 1)
    A, B = ring - [0, w / 2, 0], ring + [0, w / 2, 0]
    ca, cb = np.array([0, dy - w / 2, 0.0]), np.array([0, dy + w / 2, 0.0])
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [[A[i], B[j], A[j]], [A[i], B[i], B[j]], [ca, A[i], A[j]], [cb, B[j], B[i]]]
    tris = np.array(tris)                                  # outward winding
    a = np.radians(steer)
    R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    return tris @ R.T + [cx, cy, cz]


def heights(p, hand_low):
    h = p["table_z"] + p["hand_clear"] - hand_low
    return h


def amr_meshes(p, steer=0.0):
    """[(name, tris)] for the vehicle envelope, in the vehicle frame."""
    out = [("amr_deck", box(0, 0, p["deck_z"] + p["deck_t"] / 2, p["body_len"], p["body_w"], p["deck_t"])),
           ("amr_tub", box(0, 0, (p["tub_bottom"] + p["deck_z"]) / 2, p["tub_len"], p["tub_w"], p["deck_z"] - p["tub_bottom"]))]
    bx, bd = (p["body_len"] + p["tub_len"]) / 4, (p["body_len"] - p["tub_len"]) / 2   # bumpers fill tub end .. body end
    if bd > 1.0:
        out += [("amr_bumper_front", box(bx, 0, 330, bd, p["tub_w"] + 120, 120)),
                ("amr_bumper_rear", box(-bx, 0, 330, bd, p["tub_w"] + 120, 120))]
    for name, x in (("front", p["wheelbase"] / 2), ("rear", -p["wheelbase"] / 2)):
        for side, s in (("left", 1), ("right", -1)):
            st = steer if name == "front" else 0.0
            out.append((f"amr_wheel_{name}_{side}", wheel(x, s * p["track"] / 2, p["wheel_r"], p["wheel_r"], p["tire_w"], st)))
            out.append((f"amr_hub_{name}_{side}", wheel(x, s * p["track"] / 2, p["wheel_r"], 0.5 * p["wheel_r"], 4.0, st,
                                                        dy=s * (p["tire_w"] / 2 + 2.0))))
    return out


def adapter_meshes(p, base_z):
    """[(name, tris)] for the lower-body adapter, in the vehicle frame.

    base_z: height of the robot base origin above the ground."""
    off = np.array([p["mount_x"], 0.0, base_z])
    zp, t = p["adapter_z"], p["plate_t"]
    out = []
    u = np.array([-HIP_AX[2], 0.0, HIP_AX[0]])            # in the flange plane, up and back
    for side, sy in (("left", 1), ("right", -1)):
        J = HIP_J * [1, sy, 1] - HIP_LIP * HIP_AX
        face = {(i, j): J + (i - 0.5) * p["flange"] * np.array([0, 1.0, 0]) + (j - 0.5) * p["flange"] * u
                for i in (0, 1) for j in (0, 1)}
        V = {}
        for (i, j), q in face.items():
            V[(i, j, 1)] = q + off
            V[(i, j, 0)] = np.array([q[0], q[1], zp]) + off
        out.append((f"adapter_hip_wedge_{side}", _hexa(V)))
    x0, x1 = p["adapter_x0"], p["adapter_x1"]
    cx = (x0 + x1) / 2
    out.append(("adapter_plate", box(cx, 0, zp - t / 2, x1 - x0, p["adapter_w"], t) + off))
    deck_top = p["deck_z"] + p["deck_t"]
    out.append(("adapter_deck_plate", box(cx + p["mount_x"], 0, deck_top + t / 2, p["base_plate_x"], p["base_plate_y"], t)))
    top, bot = zp - t + base_z, deck_top + t
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append((f"adapter_post_{'front' if sx > 0 else 'rear'}_{'left' if sy > 0 else 'right'}",
                        box(cx + p["mount_x"] + sx * p["post_dx"], sy * p["post_dy"], (top + bot) / 2, p["post"], p["post"], top - bot)))
    return out


def robot_meshes(urdf, pose, offset):
    """[(link/idx, tris)] of every visual, posed, in mm, shifted by offset."""
    root = ET.parse(urdf).getroot()
    T, _ = m.link_transforms(root, pose)
    out = []
    for link in root.findall("link"):
        name = link.get("name")
        for k, v in enumerate(link.findall("visual")):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            path = m.resolve_mesh(mesh.get("filename"))
            s = np.array([float(x) for x in mesh.get("scale", "1 1 1").split()])
            tris = m.read_stl(path) * s
            Tw = T[name] @ m.origin_T(v.find("origin"))
            pts = (tris.reshape(-1, 3) @ Tw[:3, :3].T + Tw[:3, 3]) * 1000.0 + offset
            if np.prod(s) < 0:
                pts = pts.reshape(-1, 3, 3)[:, ::-1].reshape(-1, 3)
            out.append((f"{name}/{k}", pts.reshape(-1, 3, 3)))
    return out


def build(p, urdf, with_robot=True, steer=0.0):
    """Returns (amr, adapter, robot, dims). robot is [] when with_robot is False."""
    pose_robot = robot_meshes(urdf, WORK_POSE, np.zeros(3))
    hand_low = min(t[:, :, 2].min() for n, t in pose_robot if not n.startswith(("base/", "head", "zed")))
    torso_low = min(t[:, :, 2].min() for n, t in pose_robot if n.startswith("base/"))
    T, _ = m.link_transforms(ET.parse(urdf).getroot(), WORK_POSE)
    frame_z = lambda link: T[link][2, 3] * 1000.0
    base_z = heights(p, hand_low)
    robot = [(n, t + [p["mount_x"], 0, base_z]) for n, t in pose_robot] if with_robot else []
    amr = amr_meshes(p, steer)
    adapter = adapter_meshes(p, base_z)
    allr = np.concatenate([t.reshape(-1, 3) for _, t in pose_robot])
    deck_top = p["deck_z"] + p["deck_t"]
    tip_x = allr[:, 0].max() + p["mount_x"]
    dims = {
        "base_origin_z": base_z,
        "deck_top_z": deck_top,
        "adapter_plate_top_z": base_z + p["adapter_z"],
        "adapter_post_len": base_z + p["adapter_z"] - p["plate_t"] - (deck_top + p["plate_t"]),
        "torso_bottom_z": base_z + torso_low,
        "hip_flange_z": base_z + HIP_J[2],
        "hand_lowest_z": base_z + hand_low,
        "shoulder_axis_z": base_z + frame_z("arm_right_shoulder_pitch"),
        "camera_centre_z": base_z + frame_z("zed_camera_center"),
        "top_z": base_z + allr[:, 2].max(),
        "fingertip_x": tip_x,
        "fingertip_behind_front_edge": p["body_len"] / 2 - tip_x,
        "hand_above_deck": base_z + hand_low - deck_top,
    }
    return amr, adapter, robot, dims


def interference(solids, others, tol=0.98):
    """Vertices of `others` inside any convex solid (shrunk by tol). Needs scipy."""
    from scipy.spatial import Delaunay
    v = np.concatenate([t.reshape(-1, 3) for _, t in others])
    hits = {}
    for name, t in solids:
        pts = t.reshape(-1, 3)
        c = pts.mean(0)
        n = int((Delaunay(c + (pts - c) * tol).find_simplex(v) >= 0).sum())
        if n:
            hits[name] = n
    return hits


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=os.path.join(m.PKG_DIR, "urdf", "berkeley_humanoid_lite_v1arm.urdf"))
    ap.add_argument("--outdir", default=os.path.join(m.REPO_ROOT, "export"))
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE")
    ap.add_argument("--steer", type=float, default=0.0, help="front wheel angle, deg")
    ap.add_argument("--no-robot", action="store_true", help="skip the full assembly STL")
    a = ap.parse_args()
    p = {k: v for k, (v, _) in PARAMS.items()}
    src = {k: s for k, (_, s) in PARAMS.items()}
    for kv in a.set:
        k, v = kv.split("=")
        if k not in p:
            raise SystemExit(f"unknown parameter {k}; known: {', '.join(p)}")
        p[k], src[k] = float(v), "set on the command line"
    if abs(a.steer) > p["steer_max_deg"]:
        raise SystemExit(f"--steer beyond +/-{p['steer_max_deg']} deg")

    amr, adapter, robot, dims = build(p, a.urdf, not a.no_robot, a.steer)
    os.makedirs(a.outdir, exist_ok=True)
    stem = "erp42_amr"
    files = {
        f"{stem}_envelope_with_adapter_mm.stl": amr + adapter,
        f"{stem}_lower_body_adapter_mm.stl": [(n, t - [p["mount_x"], 0, dims["base_origin_z"]]) for n, t in adapter
                                             if not n.startswith("adapter_post") and n != "adapter_deck_plate"],
    }
    if robot:
        files[f"{stem}_v1arm_mobile_manipulator_work_pose_mm.stl"] = robot + amr + adapter
    for fn, parts in files.items():
        tris = np.concatenate([t for _, t in parts])
        m.write_stl(os.path.join(a.outdir, fn), tris, f"{fn} vehicle frame mm".encode())
        print(f"wrote {fn}  triangles={len(tris)}")
    clash = {"skipped": "--no-robot"}
    if robot:
        try:
            clash = interference(amr + adapter, robot)
        except ImportError:
            clash = {"skipped": "scipy not installed"}
    info = {
        "frame": "vehicle: origin on the ground under the wheelbase centre, x forward, z up, mm "
                 "(the lower_body_adapter STL alone is in the robot base frame and holds only the "
                 "wedges and the plate; the posts follow the deck height)",
        "pose": WORK_POSE,
        "params": {k: {"value": p[k], "source": src[k]} for k in p},
        "dims_mm": {k: round(float(v), 1) for k, v in dims.items()},
        "interference_vertices": clash,
        "parts": {fn: [n for n, _ in parts] for fn, parts in files.items()},
    }
    with open(os.path.join(a.outdir, f"{stem}_layout.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=1, ensure_ascii=False)
    for k, v in info["dims_mm"].items():
        print(f"  {k:30s} {v:8.1f}")
    print("interference:", clash or "none")


if __name__ == "__main__":
    main()
