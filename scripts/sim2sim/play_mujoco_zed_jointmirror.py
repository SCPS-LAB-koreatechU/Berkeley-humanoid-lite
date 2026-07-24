"""Full-body per-joint MIRROR of a human onto Berkeley Humanoid Lite (MuJoCo).

Kinematic mirror (NO RL policy, NO physics): the robot base is pinned and the 22
joints are driven directly from the ZED body-tracking stream so the robot copies
the operator's pose. Because there is no balance controller this is a *fixed-base*
visualization — the robot will not (and cannot) fall; it just mirrors you.

Retargeting strategy
--------------------
ZED per-joint *rotations* are noisy and frame-ambiguous, but the joint *positions*
(smpl_joints, 24x3 in the Unity LEFT_HANDED_Y_UP frame, metres) are the reliable
signal. So we build a torso frame from the keypoints and read each robot joint
angle from limb geometry:
  - elbow / knee  : flexion = angle between the two adjacent limb segments
  - shoulder / hip: elevation (pitch) + abduction (roll) of the upper segment,
                    decomposed in the torso frame
shoulder_yaw, elbow_roll, hip_yaw and the ankles are left at neutral (positions
alone don't observe them well).

Per-joint GAIN/SIGN/OFFSET (below) make it tunable; `--debug` prints the angles
and `--flip a,b,c` negates named joints live so you can fix any that mirror wrong.

Run (WSL), with the Windows ZED publisher streaming on :5556:
  python scripts/sim2sim/play_mujoco_zed_jointmirror.py --host 172.26.0.1
"""
import argparse
import json
import os
import time

import numpy as np
import zmq
import mujoco
import mujoco.viewer


# --- SMPL-24 indices (must match common.SMPL_24_NAMES on the publisher) -------
PELVIS, L_HIP, R_HIP, SPINE1, L_KNEE, R_KNEE = 0, 1, 2, 3, 4, 5
L_ANKLE, R_ANKLE, NECK = 7, 8, 12
L_SHO, R_SHO, L_ELB, R_ELB, L_WRI, R_WRI = 16, 17, 18, 19, 20, 21

# --- BHL joints, in config / qpos order (arm_l 5, arm_r 5, leg_l 6, leg_r 6) --
JOINT_NAMES = [
    "arm_left_shoulder_pitch_joint", "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint", "arm_left_elbow_pitch_joint",
    "arm_left_elbow_roll_joint",
    "arm_right_shoulder_pitch_joint", "arm_right_shoulder_roll_joint",
    "arm_right_shoulder_yaw_joint", "arm_right_elbow_pitch_joint",
    "arm_right_elbow_roll_joint",
    "leg_left_hip_roll_joint", "leg_left_hip_yaw_joint", "leg_left_hip_pitch_joint",
    "leg_left_knee_pitch_joint", "leg_left_ankle_pitch_joint", "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint", "leg_right_hip_yaw_joint", "leg_right_hip_pitch_joint",
    "leg_right_knee_pitch_joint", "leg_right_ankle_pitch_joint", "leg_right_ankle_roll_joint",
]
# joint limits straight from the MJCF (rad)
JOINT_RANGE = {
    "arm_left_shoulder_pitch_joint": (-1.5708, 0.7854),
    "arm_left_shoulder_roll_joint": (-0.2618, 1.3090),
    "arm_left_shoulder_yaw_joint": (-0.7854, 0.7854),
    "arm_left_elbow_pitch_joint": (0.0, 1.5708),
    "arm_left_elbow_roll_joint": (-0.7854, 0.7854),
    "arm_right_shoulder_pitch_joint": (-0.7854, 1.5708),
    "arm_right_shoulder_roll_joint": (-1.3090, 0.2618),
    "arm_right_shoulder_yaw_joint": (-0.7854, 0.7854),
    "arm_right_elbow_pitch_joint": (-1.5708, 0.0),
    "arm_right_elbow_roll_joint": (-0.7854, 0.7854),
    "leg_left_hip_roll_joint": (-0.1745, 1.5708),
    "leg_left_hip_yaw_joint": (-0.9817, 0.5890),
    "leg_left_hip_pitch_joint": (-1.8980, 0.9817),
    "leg_left_knee_pitch_joint": (0.0, 2.4435),
    "leg_left_ankle_pitch_joint": (-0.7854, 0.7854),
    "leg_left_ankle_roll_joint": (-0.2618, 0.2618),
    "leg_right_hip_roll_joint": (-1.5708, 0.1745),
    "leg_right_hip_yaw_joint": (-0.5890, 0.9817),
    "leg_right_hip_pitch_joint": (-1.8980, 0.9817),
    "leg_right_knee_pitch_joint": (0.0, 2.4435),
    "leg_right_ankle_pitch_joint": (-0.7854, 0.7854),
    "leg_right_ankle_roll_joint": (-0.2618, 0.2618),
}
# Per-joint output sign (flip if a joint mirrors the wrong way). 0 = hold neutral.
SIGN = {n: 1.0 for n in JOINT_NAMES}
SIGN.update({
    "arm_left_shoulder_yaw_joint": 0.0, "arm_left_elbow_roll_joint": 0.0,
    "arm_right_shoulder_yaw_joint": 0.0, "arm_right_elbow_roll_joint": 0.0,
    "leg_left_hip_yaw_joint": 0.0, "leg_left_ankle_pitch_joint": 0.0,
    "leg_left_ankle_roll_joint": 0.0,
    "leg_right_hip_yaw_joint": 0.0, "leg_right_ankle_pitch_joint": 0.0,
    "leg_right_ankle_roll_joint": 0.0,
})

MJCF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..",
    "source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid",
    "berkeley_humanoid_lite/mjcf/bhl_scene.xml",
)


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else None


def _decode_pose(msg: bytes):
    """{field_name: ndarray} from one ZMQ pose message (Protocol v3)."""
    _DT = {"f32": np.float32, "f64": np.float64, "i64": np.int64, "i32": np.int32,
           "float32": np.float32, "float64": np.float64, "int64": np.int64, "int32": np.int32}
    if msg[:4] == b"pose":
        msg = msg[4:]
    header = json.loads(msg[:1280].split(b"\x00", 1)[0].decode())
    off, out = 1280, {}
    for f in header["fields"]:
        shp = tuple(f["shape"])
        dt = _DT[f["dtype"]]
        nb = int(np.prod(shp)) * np.dtype(dt).itemsize
        out[f["name"]] = np.frombuffer(msg[off:off + nb], dtype=dt).reshape(shp)
        off += nb
    return out


def retarget(J, fwd_sign=1.0):
    """SMPL-24 keypoints (24,3) -> {joint_name: angle_rad}. None if pose invalid."""
    up = _unit(J[NECK] - J[PELVIS])
    lat = _unit(J[L_SHO] - J[R_SHO])          # points toward the left shoulder
    if up is None or lat is None:
        return None
    fwd = _unit(np.cross(lat, up))
    if fwd is None:
        return None
    fwd = fwd * fwd_sign
    lat = _unit(np.cross(up, fwd))            # re-orthonormalize
    if lat is None:
        return None

    out = {n: 0.0 for n in JOINT_NAMES}

    def arm(side, sho, elb, wri):
        s = +1.0 if side == "left" else -1.0   # outward = +lat (L) / -lat (R)
        upper = _unit(J[elb] - J[sho])
        fore = _unit(J[wri] - J[elb])
        if upper is None or fore is None:
            return
        flex = np.arccos(np.clip(np.dot(upper, fore), -1.0, 1.0))
        c_up, c_fwd, c_lat = np.dot(upper, up), np.dot(upper, fwd), np.dot(upper, lat)
        pitch = np.arctan2(c_fwd, -c_up)        # 0 = arm hanging down, +fwd raise
        roll = np.arcsin(np.clip(s * c_lat, -1.0, 1.0))  # abduction out to the side
        out[f"arm_{side}_shoulder_pitch_joint"] = pitch
        out[f"arm_{side}_shoulder_roll_joint"] = s * roll
        out[f"arm_{side}_elbow_pitch_joint"] = s * flex

    def leg(side, hip, knee, ankle):
        s = +1.0 if side == "left" else -1.0
        thigh = _unit(J[knee] - J[hip])
        shank = _unit(J[ankle] - J[knee])
        if thigh is None or shank is None:
            return
        flex = np.arccos(np.clip(np.dot(thigh, shank), -1.0, 1.0))
        t_up, t_fwd, t_lat = np.dot(thigh, up), np.dot(thigh, fwd), np.dot(thigh, lat)
        out[f"leg_{side}_hip_pitch_joint"] = np.arctan2(t_fwd, -t_up)   # raise knee fwd
        out[f"leg_{side}_hip_roll_joint"] = s * np.arcsin(np.clip(s * t_lat, -1.0, 1.0))
        out[f"leg_{side}_knee_pitch_joint"] = flex

    arm("left", L_SHO, L_ELB, L_WRI)
    arm("right", R_SHO, R_ELB, R_WRI)
    leg("left", L_HIP, L_KNEE, L_ANKLE)
    leg("right", R_HIP, R_KNEE, R_ANKLE)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="172.26.0.1")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--topic", default="pose")
    ap.add_argument("--fwd-sign", type=float, default=1.0, help="flip if fwd/back inverted")
    ap.add_argument("--ema", type=float, default=0.4, help="smoothing 0..1 (higher=snappier)")
    ap.add_argument("--flip", default="", help="comma list of joints to negate")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    for j in [s for s in args.flip.split(",") if s]:
        if j in SIGN:
            SIGN[j] *= -1.0
        else:
            print(f"[mirror] WARN unknown --flip joint: {j}")

    model = mujoco.MjModel.from_xml_path(os.path.normpath(MJCF))
    data = mujoco.MjData(model)
    qadr = {n: model.joint(n).qposadr[0] for n in JOINT_NAMES}
    base_adr = model.joint("base_freejoint").qposadr[0]

    # ground the fixed base: set default pose, forward-kinematics, drop feet to floor
    data.qpos[base_adr:base_adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    min_z = min(data.geom_xpos[g][2] for g in range(model.ngeom))
    data.qpos[base_adr + 2] = -min_z + 0.02

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://{args.host}:{args.port}")
    sock.setsockopt(zmq.SUBSCRIBE, args.topic.encode())
    sock.setsockopt(zmq.RCVTIMEO, 200)
    print(f"[mirror] subscribing tcp://{args.host}:{args.port} topic={args.topic!r} — "
          f"full-body kinematic mirror (fixed base)")

    smooth = {n: 0.0 for n in JOINT_NAMES}
    a = float(np.clip(args.ema, 0.05, 1.0))
    last_dbg = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            try:
                msg = sock.recv()
            except zmq.Again:
                viewer.sync()
                continue
            try:
                fields = _decode_pose(msg)
                J = np.asarray(fields["smpl_joints"], dtype=np.float64).reshape(-1, 3)
            except Exception:
                continue
            if J.shape[0] < 24 or not np.all(np.isfinite(J[[PELVIS, NECK, L_SHO, R_SHO]])):
                continue

            angles = retarget(J, fwd_sign=args.fwd_sign)
            if angles is None:
                continue

            for n in JOINT_NAMES:
                lo, hi = JOINT_RANGE[n]
                target = float(np.clip(SIGN[n] * angles[n], lo, hi))
                smooth[n] = (1 - a) * smooth[n] + a * target
                data.qpos[qadr[n]] = smooth[n]

            mujoco.mj_forward(model, data)      # kinematics only, no physics step
            viewer.sync()

            if args.debug and time.time() - last_dbg > 0.5:
                last_dbg = time.time()
                print("  ".join(f"{n.split('_joint')[0].replace('arm_','').replace('leg_','')}"
                                 f"={smooth[n]:+.2f}" for n in JOINT_NAMES[:8]))


if __name__ == "__main__":
    main()
