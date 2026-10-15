"""Merge every <visual> mesh of a URDF into one STL at a fixed joint pose.

Pure numpy (no trimesh / ROS needed). package:// paths are resolved with the
same PACKAGE_MAP idea as blender_import_urdf.py, because the meshes,
meshes_v1, meshes_dexhand symlinks do not survive a Windows checkout.

Usage (from anywhere):
    python3 merge_urdf_stl.py                       # v1arm, zero pose, mm
    python3 merge_urdf_stl.py --urdf ../urdf/berkeley_humanoid_lite_dexhand.urdf
    python3 merge_urdf_stl.py --units m --out my.stl
    python3 merge_urdf_stl.py --pose pose.json      # {"joint_name": rad, ...}
    python3 merge_urdf_stl.py --subtree head_mount --frame head_mount   # head only

Outputs <out>.stl (binary) and <out>_parts.json (per-part triangle count,
world bbox, source file) next to it.
"""

import argparse
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.normpath(os.path.join(HERE, ".."))
REPO_ROOT = os.path.normpath(os.path.join(PKG_DIR, "..", "..", ".."))

_PKG = "package://berkeley_humanoid_lite_description/"
# longest prefix first
PACKAGE_MAP = [
    ("meshes_dexhand_left", os.path.join(PKG_DIR, "meshes_dexhand_left")),
    ("meshes_dexhand", os.path.join(REPO_ROOT, "ros2_ws", "vendor", "dexhandv2_description", "meshes")),
    ("meshes_v1", os.path.join(REPO_ROOT, "ros2_ws", "vendor", "dexhand_v1_description", "meshes")),
    ("meshes_adapter", os.path.join(PKG_DIR, "meshes_adapter")),
    ("meshes", os.path.join(REPO_ROOT, "source", "berkeley_humanoid_lite_assets", "data",
                            "robots", "berkeley_humanoid", "berkeley_humanoid_lite", "meshes")),
]


def resolve_mesh(filename):
    if filename.startswith(_PKG):
        rel = filename[len(_PKG):]
        for prefix, base in PACKAGE_MAP:
            if rel.startswith(prefix + "/"):
                return os.path.normpath(os.path.join(base, *rel[len(prefix) + 1:].split("/")))
        return os.path.normpath(os.path.join(PKG_DIR, *rel.split("/")))
    if filename.startswith("file://"):
        return filename[len("file://"):]
    return filename


# ---------------------------------------------------------------- math
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def origin_T(elem):
    T = np.eye(4)
    if elem is None:
        return T
    xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_T(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


# ---------------------------------------------------------------- STL io
def read_stl(path):
    """Return (N,3,3) float64 triangle array."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([
                ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
            return rec["v"].astype(np.float64)
    # ASCII fallback
    verts = []
    for line in data.decode("ascii", errors="ignore").splitlines():
        s = line.strip()
        if s.startswith("vertex"):
            verts.append([float(x) for x in s.split()[1:4]])
    return np.asarray(verts, float).reshape(-1, 3, 3)


def write_stl(path, tris, header=b""):
    tris = tris.astype(np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 0)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"] = nrm
    rec["v"] = tris
    with open(path, "wb") as f:
        f.write(header[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


# ---------------------------------------------------------------- FK
def link_transforms(root, pose):
    joints = root.findall("joint")
    children = {}
    child_set = set()
    for j in joints:
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        children.setdefault(p, []).append(j)
        child_set.add(c)
    links = [l.get("name") for l in root.findall("link")]
    roots = [l for l in links if l not in child_set]
    if len(roots) != 1:
        sys.exit(f"expected one root link, got {roots}")

    # resolve mimic joints
    jmap = {j.get("name"): j for j in joints}
    q = dict(pose)
    for j in joints:
        m = j.find("mimic")
        if m is not None and j.get("name") not in pose:
            src = m.get("joint")
            q[j.get("name")] = float(m.get("multiplier", 1)) * q.get(src, 0.0) + float(m.get("offset", 0))

    T = {roots[0]: np.eye(4)}
    stack = [roots[0]]
    while stack:
        p = stack.pop()
        for j in children.get(p, []):
            c = j.find("child").get("link")
            Tj = T[p] @ origin_T(j.find("origin"))
            jt = j.get("type")
            qv = q.get(j.get("name"), 0.0)
            if jt in ("revolute", "continuous") and qv != 0.0:
                ax = [float(v) for v in j.find("axis").get("xyz").split()] if j.find("axis") is not None else [1, 0, 0]
                Tj = Tj @ axis_angle_T(ax, qv)
            elif jt == "prismatic" and qv != 0.0:
                ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                Tp = np.eye(4); Tp[:3, 3] = ax / np.linalg.norm(ax) * qv
                Tj = Tj @ Tp
            T[c] = Tj
            stack.append(c)
    return T, roots[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=os.path.join(PKG_DIR, "urdf", "berkeley_humanoid_lite_v1arm.urdf"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--units", choices=["mm", "m"], default="mm")
    ap.add_argument("--pose", default=None, help="json {joint: rad}; default all zero")
    ap.add_argument("--geometry", choices=["visual", "collision"], default="visual")
    ap.add_argument("--subtree", default=None,
                    help="only this link and everything below it (e.g. head_mount)")
    ap.add_argument("--frame", default=None,
                    help="express the result in this link's frame; default the root")
    a = ap.parse_args()

    urdf = os.path.abspath(a.urdf)
    stem = os.path.splitext(os.path.basename(urdf))[0]
    tag = f"_{a.subtree}" if a.subtree else ""
    out = a.out or os.path.join(REPO_ROOT, "export", f"{stem}{tag}_zero_pose_{a.units}.stl")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    pose = json.load(open(a.pose)) if a.pose else {}
    unit = 1000.0 if a.units == "mm" else 1.0

    root = ET.parse(urdf).getroot()
    T, root_link = link_transforms(root, pose)
    frame = a.frame or root_link
    if frame not in T:
        sys.exit(f"unknown --frame link: {frame}")
    to_frame = np.linalg.inv(T[frame])
    keep = None
    if a.subtree:
        if a.subtree not in T:
            sys.exit(f"unknown --subtree link: {a.subtree}")
        below = {}
        for j in root.findall("joint"):
            below.setdefault(j.find("parent").get("link"), []).append(j.find("child").get("link"))
        keep, todo = set(), [a.subtree]
        while todo:
            name = todo.pop()
            keep.add(name)
            todo.extend(below.get(name, []))

    all_tris, parts, missing = [], [], []
    for link in root.findall("link"):
        name = link.get("name")
        if keep is not None and name not in keep:
            continue
        for k, v in enumerate(link.findall(a.geometry)):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            path = resolve_mesh(mesh.get("filename"))
            if not os.path.isfile(path):
                missing.append(path)
                continue
            s = np.array([float(x) for x in mesh.get("scale", "1 1 1").split()])
            tris = read_stl(path) * s
            Tw = to_frame @ T[name] @ origin_T(v.find("origin"))
            pts = tris.reshape(-1, 3) @ Tw[:3, :3].T + Tw[:3, 3]
            if np.linalg.det(np.diag(s)) < 0:  # mirrored scale flips winding
                pts = pts.reshape(-1, 3, 3)[:, ::-1].reshape(-1, 3)
            pts *= unit
            tris_w = pts.reshape(-1, 3, 3)
            all_tris.append(tris_w)
            flat = tris_w.reshape(-1, 3)
            parts.append({
                "link": name, "visual_index": k,
                "source": os.path.relpath(path, REPO_ROOT).replace(os.sep, "/"),
                "triangles": int(len(tris_w)),
                "bbox_min": flat.min(0).round(3).tolist(), "bbox_max": flat.max(0).round(3).tolist(),
            })

    if missing:
        print("MISSING meshes:", *missing, sep="\n  ")
    merged = np.concatenate(all_tris)
    header = f"{stem}{tag} {a.geometry} pose={'zero' if not pose else 'custom'} units={a.units} frame={frame}".encode()
    write_stl(out, merged, header)

    flat = merged.reshape(-1, 3)
    lo, hi = flat.min(0), flat.max(0)
    summary = {
        "urdf": os.path.relpath(urdf, REPO_ROOT).replace(os.sep, "/"),
        "geometry": a.geometry, "units": a.units, "frame": frame,
        "subtree": a.subtree or root_link,
        "pose": pose or "all joints 0",
        "parts": len(parts), "triangles": int(len(merged)), "missing": missing,
        "bbox_min": lo.round(3).tolist(), "bbox_max": hi.round(3).tolist(),
        "size": (hi - lo).round(3).tolist(),
        "part_list": parts,
    }
    with open(os.path.splitext(out)[0] + "_parts.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1, ensure_ascii=False)
    print(f"wrote {out}")
    print(f"parts={len(parts)} triangles={len(merged)} missing={len(missing)}")
    print(f"bbox [{a.units}] min={lo.round(1).tolist()} max={hi.round(1).tolist()} size={(hi - lo).round(1).tolist()}")


if __name__ == "__main__":
    main()
