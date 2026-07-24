"""Shared schema + (de)serialization for the ZED 2i -> SONIC teleop bridge.

This module is imported by BOTH sides:
  - Windows publisher (zed_publisher_windows.py), running with the ZED SDK.
  - WSL subscriber (zed_reader.py / test_receive.py), feeding SONIC.

Wire format (ZMQ PUB/SUB, multipart):
  frame[0] = TOPIC (bytes)                       e.g. b"zed.smpl24"
  frame[1] = meta JSON (utf-8)                   {"ts_ns", "fps", "tracked", "seq"}
  frame[2] = body_poses float32 C-contig bytes   shape (24, 7) = SMPL-24 [x,y,z, qx,qy,qz,qw]

Contract for body_poses (MUST match SONIC's PicoReader output):
  - 24 SMPL joints, standard SMPL ordering (see SMPL_24_NAMES).
  - Each row: position (x,y,z) in METERS + GLOBAL orientation quaternion in
    scalar-LAST (qx,qy,qz,qw) order.
  - Coordinate frame: Unity == LEFT_HANDED_Y_UP (Y up, Z forward, left-handed).
    The SONIC manager (pico_manager_thread_server._process_3pt_pose) converts
    Unity -> robot frame itself, so we deliver Unity frame here.

Verified against the SONIC repo:
  gear_sonic/scripts/pico_manager_thread_server.py
    - line ~211: "smpl_pose_np: (24,7) - 24 SMPL joints, each [x,y,z,qx,qy,qz,qw] in Unity frame (scalar-last)"
    - line ~566: global_quats = body_poses_np[:, [6,3,4,5]]   (xyzw -> wxyz)
"""

from __future__ import annotations

import json
import struct

import numpy as np

TOPIC = b"zed.smpl24"
DEFAULT_PORT = 5599

# --- SMPL-24 canonical joint order (what SONIC expects) ----------------------
SMPL_24_NAMES = [
    "pelvis",        # 0  (root)
    "left_hip",      # 1
    "right_hip",     # 2
    "spine1",        # 3
    "left_knee",     # 4
    "right_knee",    # 5
    "spine2",        # 6
    "left_ankle",    # 7
    "right_ankle",   # 8
    "spine3",        # 9
    "left_foot",     # 10
    "right_foot",    # 11
    "neck",          # 12
    "left_collar",   # 13
    "right_collar",  # 14
    "head",          # 15
    "left_shoulder", # 16
    "right_shoulder",# 17
    "left_elbow",    # 18
    "right_elbow",   # 19
    "left_wrist",    # 20
    "right_wrist",   # 21
    "left_hand",     # 22
    "right_hand",    # 23
]
NUM_SMPL = 24

# --- ZED SDK BODY_38 part indices (ZED SDK 4.x BODY_38_PARTS) -----------------
# NOTE: verify against your installed SDK with:
#   python -c "import pyzed.sl as sl; print(list(sl.BODY_38_PARTS.__members__))"
# Index list below is the documented BODY_38 ordering; core body joints (0-29)
# are stable. Hand keypoints (30-37) are only used as fallbacks for SMPL hands.
ZED38 = {
    "PELVIS": 0, "SPINE_1": 1, "SPINE_2": 2, "SPINE_3": 3, "NECK": 4,
    "NOSE": 5, "LEFT_EYE": 6, "RIGHT_EYE": 7, "LEFT_EAR": 8, "RIGHT_EAR": 9,
    "LEFT_CLAVICLE": 10, "RIGHT_CLAVICLE": 11,
    "LEFT_SHOULDER": 12, "RIGHT_SHOULDER": 13,
    "LEFT_ELBOW": 14, "RIGHT_ELBOW": 15,
    "LEFT_WRIST": 16, "RIGHT_WRIST": 17,
    "LEFT_HIP": 18, "RIGHT_HIP": 19,
    "LEFT_KNEE": 20, "RIGHT_KNEE": 21,
    "LEFT_ANKLE": 22, "RIGHT_ANKLE": 23,
    "LEFT_BIG_TOE": 24, "RIGHT_BIG_TOE": 25,
    "LEFT_SMALL_TOE": 26, "RIGHT_SMALL_TOE": 27,
    "LEFT_HEEL": 28, "RIGHT_HEEL": 29,
    # hand keypoints (names differ slightly across SDK minor versions):
    "LEFT_HAND_THUMB_4": 30, "RIGHT_HAND_THUMB_4": 31,
    "LEFT_HAND_INDEX_1": 32, "RIGHT_HAND_INDEX_1": 33,
    "LEFT_HAND_MIDDLE_4": 34, "RIGHT_HAND_MIDDLE_4": 35,
    "LEFT_HAND_PINKY_1": 36, "RIGHT_HAND_PINKY_1": 37,
}

# --- ZED BODY_38 -> SMPL-24 position mapping ---------------------------------
# smpl_index -> zed38_index. Where SMPL has no exact ZED match we approximate
# (head<-nose, feet<-big toe, hands<-wrist). >>> VERIFY EMPIRICALLY with camera. <<<
SMPL_FROM_ZED = {
    0:  ZED38["PELVIS"],
    1:  ZED38["LEFT_HIP"],
    2:  ZED38["RIGHT_HIP"],
    3:  ZED38["SPINE_1"],
    4:  ZED38["LEFT_KNEE"],
    5:  ZED38["RIGHT_KNEE"],
    6:  ZED38["SPINE_2"],
    7:  ZED38["LEFT_ANKLE"],
    8:  ZED38["RIGHT_ANKLE"],
    9:  ZED38["SPINE_3"],
    10: ZED38["LEFT_BIG_TOE"],
    11: ZED38["RIGHT_BIG_TOE"],
    12: ZED38["NECK"],
    13: ZED38["LEFT_CLAVICLE"],
    14: ZED38["RIGHT_CLAVICLE"],
    15: ZED38["NOSE"],            # approximation for head
    16: ZED38["LEFT_SHOULDER"],
    17: ZED38["RIGHT_SHOULDER"],
    18: ZED38["LEFT_ELBOW"],
    19: ZED38["RIGHT_ELBOW"],
    20: ZED38["LEFT_WRIST"],
    21: ZED38["RIGHT_WRIST"],
    22: ZED38["LEFT_WRIST"],      # SMPL "hand" ~ wrist if no hand kp; refine to LEFT_HAND_*
    23: ZED38["RIGHT_WRIST"],
}

IDENTITY_QUAT_XYZW = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)


def pack(body_poses: np.ndarray, ts_ns: int, fps: float, tracked: bool, seq: int) -> list[bytes]:
    """Return ZMQ multipart frames for a (24,7) float32 SMPL pose."""
    arr = np.ascontiguousarray(body_poses, dtype=np.float32)
    assert arr.shape == (NUM_SMPL, 7), f"expected (24,7), got {arr.shape}"
    meta = json.dumps({"ts_ns": int(ts_ns), "fps": float(fps),
                       "tracked": bool(tracked), "seq": int(seq)}).encode("utf-8")
    return [TOPIC, meta, arr.tobytes(order="C")]


def unpack(frames: list[bytes]) -> tuple[dict, np.ndarray]:
    """Inverse of pack(). Returns (meta_dict, (24,7) float32 array)."""
    _topic, meta_b, arr_b = frames[0], frames[1], frames[2]
    meta = json.loads(meta_b.decode("utf-8"))
    arr = np.frombuffer(arr_b, dtype=np.float32).reshape(NUM_SMPL, 7).copy()
    return meta, arr


# self-test
if __name__ == "__main__":
    a = np.zeros((24, 7), dtype=np.float32)
    a[:, 6] = 1.0  # qw=1
    m, b = unpack(pack(a, 123, 60.0, True, 1))
    assert b.shape == (24, 7) and m["fps"] == 60.0
    print("common.py self-test OK:", m, b.shape)
