"""ZED pose -> velocity command pad for Berkeley Humanoid Lite sim2sim.

Drop-in replacement for the USB-gamepad `Se2Gamepad`: exposes a `.commands`
dict ({velocity_x, velocity_y, velocity_yaw, mode_switch}) that a background
thread continuously updates from the ZED body-tracking ZMQ stream.

Teleop scheme = "lean to walk":
  - lean torso forward/back  -> velocity_x  (walk fwd/back)
  - lean torso left/right    -> velocity_y  (strafe)
  - turn body left/right     -> velocity_yaw (turn)

The neutral standing pose is auto-calibrated over the first `calib_frames`
frames, then commands are (angle - neutral) past a deadzone, scaled, clamped to
[-1, 1] and EMA-smoothed. If the stream stalls, commands decay to zero (safety).

Wire format (validated against the SONIC publisher): a single ZMQ frame =
b"pose" prefix + 1280-byte null-padded JSON header + concatenated little-endian
binary fields. Header: {"v":3,"fields":[{name,dtype,shape}, ...]}.
"""
import json
import math
import time
import threading

import numpy as np
import zmq

_DT = {
    "f32": np.float32, "f64": np.float64, "i64": np.int64, "i32": np.int32,
    "float32": np.float32, "float64": np.float64, "int64": np.int64, "int32": np.int32,
}


def _decode_pose(msg: bytes):
    """Return {field_name: np.ndarray} from one raw ZMQ pose message."""
    if msg[:4] == b"pose":
        msg = msg[4:]
    header = json.loads(msg[:1280].split(b"\x00", 1)[0].decode())
    off = 1280
    out = {}
    for f in header["fields"]:
        shp = tuple(f["shape"])
        dt = _DT[f["dtype"]]
        nb = int(np.prod(shp)) * np.dtype(dt).itemsize
        out[f["name"]] = np.frombuffer(msg[off:off + nb], dtype=dt).reshape(shp)
        off += nb
    return out


def _quat_to_euler_wxyz(q):
    """Quaternion [w,x,y,z] -> (roll, pitch, yaw) radians (ZYX)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    # roll (x-axis)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _deadzone(v, dz):
    if v > dz:
        return v - dz
    if v < -dz:
        return v + dz
    return 0.0


class ZedVelocityPad:
    def __init__(self, host="172.26.0.1", port=5556, topic=b"pose",
                 gain_x=3.0, gain_y=3.0, gain_yaw=2.0,
                 sign_x=1.0, sign_y=1.0, sign_yaw=1.0,
                 deadzone=0.06, ema=0.2, calib_frames=30,
                 stale_timeout=0.4, debug=False):
        self.host, self.port, self.topic = host, port, topic
        self.gain = (gain_x, gain_y, gain_yaw)
        self.sign = (sign_x, sign_y, sign_yaw)
        self.deadzone = deadzone
        self.ema = ema
        self.calib_frames = calib_frames
        self.stale_timeout = stale_timeout
        self.debug = debug

        # mode_switch=3 => RL control mode (matches A+RB on the real pad)
        self.commands = {
            "velocity_x": 0.0, "velocity_y": 0.0, "velocity_yaw": 0.0,
            "mode_switch": 3,
        }
        self._stopped = threading.Event()
        self._neutral = None
        self._calib_buf = []

    def run(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stopped.set()

    def _loop(self):
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.connect(f"tcp://{self.host}:{self.port}")
        sock.setsockopt(zmq.SUBSCRIBE, self.topic)
        sock.setsockopt(zmq.RCVTIMEO, 200)
        print(f"[zed-pad] subscribing tcp://{self.host}:{self.port} (lean to walk; calibrating neutral...)")
        last_rx = time.time()

        while not self._stopped.is_set():
            try:
                msg = sock.recv()
            except zmq.Again:
                # stream stalled -> decay to zero for safety
                if time.time() - last_rx > self.stale_timeout:
                    for k in ("velocity_x", "velocity_y", "velocity_yaw"):
                        self.commands[k] *= 0.5
                continue
            last_rx = time.time()

            try:
                fields = _decode_pose(msg)
                q = fields["body_quat"].reshape(-1)[:4]
            except Exception:
                continue

            roll, pitch, yaw = _quat_to_euler_wxyz(q)

            # auto-calibrate neutral standing pose
            if self._neutral is None:
                self._calib_buf.append((roll, pitch, yaw))
                if len(self._calib_buf) >= self.calib_frames:
                    self._neutral = np.mean(self._calib_buf, axis=0)
                    print(f"[zed-pad] neutral set: roll={self._neutral[0]:+.3f} "
                          f"pitch={self._neutral[1]:+.3f} yaw={self._neutral[2]:+.3f} — go!")
                continue

            dr = _deadzone(roll - self._neutral[0], self.deadzone)
            dp = _deadzone(pitch - self._neutral[1], self.deadzone)
            dy = _deadzone(yaw - self._neutral[2], self.deadzone)

            # pitch -> forward (vx), roll -> lateral (vy), yaw -> turn (vyaw)
            tx = float(np.clip(self.sign[0] * self.gain[0] * dp, -1.0, 1.0))
            ty = float(np.clip(self.sign[1] * self.gain[1] * dr, -1.0, 1.0))
            tw = float(np.clip(self.sign[2] * self.gain[2] * dy, -1.0, 1.0))

            a = self.ema
            self.commands["velocity_x"] = (1 - a) * self.commands["velocity_x"] + a * tx
            self.commands["velocity_y"] = (1 - a) * self.commands["velocity_y"] + a * ty
            self.commands["velocity_yaw"] = (1 - a) * self.commands["velocity_yaw"] + a * tw

            if self.debug:
                print(f"[zed-pad] r/p/y={roll:+.2f}/{pitch:+.2f}/{yaw:+.2f}  "
                      f"vx={self.commands['velocity_x']:+.2f} "
                      f"vy={self.commands['velocity_y']:+.2f} "
                      f"vyaw={self.commands['velocity_yaw']:+.2f}")
