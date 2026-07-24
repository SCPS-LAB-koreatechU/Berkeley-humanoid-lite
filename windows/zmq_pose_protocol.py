"""Self-contained encoder for SONIC's `--input-type zmq` wire format.

Replicates gear_sonic/utils/teleop/zmq/zmq_planner_sender.py:pack_pose_message
so publishers (incl. the Windows ZED one) need NO gear_sonic install — only
numpy. Verified against the C++ subscriber
(gear_sonic_deploy/.../zmq_packed_message_subscriber.hpp):

  message = [topic bytes][1280-byte JSON header, null-padded][binary fields]
  header  = {"v":<ver>,"endian":"le","count":1,
             "fields":[{"name","dtype","shape"}...]}   (compact json)
  field binary = C-contiguous little-endian, concatenated in header order.

Protocol versions (see docs/source/tutorials/zmq.md):
  v1 joint-based : joint_pos[N,29], joint_vel[N,29]            (+ body_quat, frame_index)
  v2 SMPL        : smpl_joints[N,24,3], smpl_pose[N,21,3]      (+ body_quat, frame_index)
  v3 joint+SMPL  : v1 fields + v2 fields (SONIC teleop; only 6 wrist joints in joint_pos need values)
Common required: body_quat[N,4] (wxyz), frame_index[N] (i64).
Subscribe topic default: "pose", port 5556.
"""
from __future__ import annotations

import json
import numpy as np

HEADER_SIZE = 1280
DEPLOY_PORT = 5556
DEPLOY_TOPIC = "pose"

_DTYPE = {np.dtype("float32"): "f32", np.dtype("float64"): "f64",
          np.dtype("int32"): "i32", np.dtype("int64"): "i64", np.dtype("bool"): "bool"}


def _build_header(fields: list, version: int, count: int = 1) -> bytes:
    header = {"v": version, "endian": "le", "count": count, "fields": fields}
    hj = json.dumps(header, separators=(",", ":")).encode("utf-8")
    if len(hj) > HEADER_SIZE:
        raise ValueError(f"Header too large: {len(hj)} > {HEADER_SIZE}")
    return hj.ljust(HEADER_SIZE, b"\x00")


def pack_pose_message(pose_data: dict, topic: str = DEPLOY_TOPIC, version: int = 3) -> bytes:
    """pose_data: {field_name: np.ndarray}. Returns the packed bytes frame."""
    fields, blobs = [], []
    for key, value in pose_data.items():
        v = np.ascontiguousarray(value)
        if v.dtype.byteorder == ">":
            v = v.astype(v.dtype.newbyteorder("<"))
        dtype_str = _DTYPE.get(v.dtype)
        if dtype_str is None:
            v = v.astype(np.float32); dtype_str = "f32"
        fields.append({"name": key, "dtype": dtype_str, "shape": list(v.shape)})
        blobs.append(v.tobytes())
    return topic.encode("utf-8") + _build_header(fields, version) + b"".join(blobs)


def decode_for_selftest(msg: bytes, topic: str = DEPLOY_TOPIC) -> dict:
    """Mirror of the C++ parser — used only by selftests to validate our packing."""
    assert msg.startswith(topic.encode()), "topic prefix mismatch"
    body = msg[len(topic):]
    hdr = body[:HEADER_SIZE]
    header = json.loads(hdr.split(b"\x00", 1)[0].decode())
    data = body[HEADER_SIZE:]
    sizes = {"f32": 4, "f64": 8, "i32": 4, "i64": 8, "bool": 1}
    npdt = {"f32": "<f4", "f64": "<f8", "i32": "<i4", "i64": "<i8", "bool": "?"}
    out, off = {}, 0
    for f in header["fields"]:
        n = int(np.prod(f["shape"])) * sizes[f["dtype"]]
        arr = np.frombuffer(data[off:off + n], dtype=npdt[f["dtype"]]).reshape(f["shape"])
        out[f["name"]] = arr; off += n
    out["__version__"] = header["v"]
    assert off == len(data), f"trailing/short data: parsed {off} of {len(data)}"
    return out
