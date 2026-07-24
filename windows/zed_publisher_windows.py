"""ZED 2i body tracking -> SONIC `--input-type zmq` (Protocol v3).  RUNS ON WINDOWS.

Camera FRONT-END. Streams the human pose from a ZED 2i directly to the SONIC
C++ controller's ZMQ input (no Python manager needed). Run where the camera is
plugged in (native Windows; ZED SDK + pyzed + CUDA on the RTX 4090).

Setup (Windows):
  1. Install ZED SDK 4.x (with CUDA) from stereolabs.com  (free account)
  2. python -m pip install pyzed pyzmq numpy
  3. copy this folder (common.py + zmq_pose_protocol.py + this file) to Windows
Run (Windows):
  python zed_publisher_windows.py --port 5556
Deploy side (WSL), point --input-type zmq at this machine:
  bash deploy.sh --input-type zmq --zmq-host <WINDOWS_IP> --zmq-port 5556 --zmq-topic pose sim
  then press ]  then ENTER (enable ZMQ streaming).

>>> MUST VERIFY WITH CAMERA <<<
  - ZED38->SMPL24 index map (common.SMPL_FROM_ZED): head/feet/hands approximated.
  - smpl_pose axis-angle: derived from ZED per-joint orientation. The SMPL pose
    convention (which 21 joints, parent-relative vs global, frame) needs a T-pose
    check against the SONIC vr3pt visualizer. Positions (smpl_joints) are reliable;
    rotations are best-effort until tuned.
  - Coordinate frame: we request LEFT_HANDED_Y_UP (Unity) from the ZED SDK.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from common import NUM_SMPL, SMPL_FROM_ZED
from zmq_pose_protocol import DEPLOY_PORT, DEPLOY_TOPIC, pack_pose_message

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
import zmq

# SMPL pose params cover 21 body joints (joints 1..21; excludes root + 2 hands).
SMPL_POSE_JOINTS = list(range(1, 22))  # 21 joints


def quat_xyzw_to_axis_angle(q: np.ndarray) -> np.ndarray:
    """xyzw quaternion -> axis-angle (3,). Identity-safe."""
    q = q / (np.linalg.norm(q) + 1e-9)
    x, y, z, w = q
    w = np.clip(w, -1.0, 1.0)
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(1.0 - w * w, 1e-12))
    if s < 1e-6:
        return np.zeros(3, dtype=np.float32)
    return (np.array([x, y, z]) / s * angle).astype(np.float32)


def build_v3_message(kp_xyz: np.ndarray, kp_quat_xyzw: np.ndarray | None,
                     root_quat_wxyz: np.ndarray, frame_idx: int) -> bytes:
    smpl_joints = np.zeros((1, NUM_SMPL, 3), dtype=np.float32)
    for smpl_i, zed_i in SMPL_FROM_ZED.items():
        if zed_i < len(kp_xyz) and np.all(np.isfinite(kp_xyz[zed_i])):
            smpl_joints[0, smpl_i] = kp_xyz[zed_i]

    smpl_pose = np.zeros((1, 21, 3), dtype=np.float32)   # axis-angle per body joint
    if kp_quat_xyzw is not None:
        for k, smpl_i in enumerate(SMPL_POSE_JOINTS):
            zed_i = SMPL_FROM_ZED.get(smpl_i)
            if zed_i is not None and zed_i < len(kp_quat_xyzw):
                q = kp_quat_xyzw[zed_i]
                if np.all(np.isfinite(q)) and np.linalg.norm(q) > 1e-6:
                    smpl_pose[0, k] = quat_xyzw_to_axis_angle(q)

    pose = {
        # v3 requires joint_pos/joint_vel; only 6 wrist joints need meaning -> zeros OK
        "joint_pos": np.zeros((1, 29), dtype=np.float32),
        "joint_vel": np.zeros((1, 29), dtype=np.float32),
        "smpl_joints": smpl_joints,
        "smpl_pose": smpl_pose,
        "body_quat": root_quat_wxyz.reshape(1, 4).astype(np.float32),
        "frame_index": np.array([frame_idx], dtype=np.int64),
    }
    return pack_pose_message(pose, topic=DEPLOY_TOPIC, version=3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=DEPLOY_PORT)
    ap.add_argument("--body-format", default="BODY_38")
    args = ap.parse_args()
    if sl is None:
        raise SystemExit("pyzed not installed. Install ZED SDK + `pip install pyzed` on Windows.")

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{args.port}")
    print(f"[zed] streaming Protocol v3 (SMPL) on tcp://*:{args.port} topic='{DEPLOY_TOPIC}'")

    cam = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.camera_fps = 60
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP   # Unity
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        raise SystemExit("Failed to open ZED camera")

    bp = sl.BodyTrackingParameters()
    bp.enable_tracking = True
    bp.enable_body_fitting = True
    bp.body_format = getattr(sl.BODY_FORMAT, args.body_format)
    bp.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
    if cam.enable_body_tracking(bp) != sl.ERROR_CODE.SUCCESS:
        raise SystemExit("Failed to enable body tracking")

    runtime = sl.BodyTrackingRuntimeParameters()
    runtime.detection_confidence_threshold = 40
    bodies = sl.Bodies()
    rt = sl.RuntimeParameters()
    seq = 0
    last_root_wxyz = np.array([1, 0, 0, 0], dtype=np.float32)   # identity until a valid root arrives
    print("[zed] stand in view; press ]+ENTER in the deploy terminal. Ctrl+C to stop")
    try:
        while True:
            if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue
            cam.retrieve_bodies(bodies, runtime)
            if not bodies.body_list:
                continue
            b = max(bodies.body_list, key=lambda x: x.confidence)
            kp = np.asarray(b.keypoint, dtype=np.float32)
            quat = (np.asarray(b.local_orientation_per_joint, dtype=np.float32)
                    if getattr(b, "local_orientation_per_joint", None) is not None else None)
            # NaN guard for keypoints (need a valid pelvis at least)
            if not np.all(np.isfinite(kp[0])):
                continue

            # global_root_orientation is xyzw -> body_quat wants wxyz.
            # It can be NaN during tracking warm-up — hold the last valid value
            # (identity until the first valid frame).
            root_q = np.asarray(getattr(b, "global_root_orientation", [0, 0, 0, 1]), dtype=np.float32)
            if root_q.size == 4 and np.all(np.isfinite(root_q)) and np.linalg.norm(root_q) > 1e-6:
                last_root_wxyz = np.array([root_q[3], root_q[0], root_q[1], root_q[2]], dtype=np.float32)
            # last_root_wxyz defaults to identity until a valid root arrives
            sock.send(build_v3_message(kp, quat, last_root_wxyz, seq))
            seq += 1
            if seq % 120 == 0:
                print(f"[zed] frame {seq}  (root valid: {np.linalg.norm(last_root_wxyz) > 0})")
    except KeyboardInterrupt:
        pass
    finally:
        cam.disable_body_tracking(); cam.close(); sock.close(0)
        print(f"[zed] stopped after {seq} frames")


if __name__ == "__main__":
    main()
