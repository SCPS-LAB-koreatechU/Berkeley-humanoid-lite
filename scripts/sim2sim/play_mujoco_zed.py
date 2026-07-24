# ZED-teleop launcher for Berkeley Humanoid Lite (MuJoCo sim2sim).
# Drives the pretrained walking policy with velocity commands derived from the
# ZED body-tracking stream (lean to walk). No SONIC controller / G1 / DDS used.
#
#   ZED pose (ZMQ :5556) -> ZedVelocityPad -> vx/vy/vyaw -> BHL MujocoSimulator
import os
import sys
import argparse

import numpy as np
import torch

# zed_velocity_pad.py lives next to this file
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from zed_velocity_pad import ZedVelocityPad

import berkeley_humanoid_lite_lowlevel.policy.gamepad as _gp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/policy_biped_50hz.yaml")
    ap.add_argument("--host", default="172.26.0.1", help="ZED publisher host (WSL->Windows gateway)")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--gain-x", type=float, default=3.0)
    ap.add_argument("--gain-y", type=float, default=3.0)
    ap.add_argument("--gain-yaw", type=float, default=2.0)
    ap.add_argument("--sign-x", type=float, default=1.0)
    ap.add_argument("--sign-y", type=float, default=1.0)
    ap.add_argument("--sign-yaw", type=float, default=1.0)
    ap.add_argument("--deadzone", type=float, default=0.06)
    ap.add_argument("--debug", action="store_true", help="print derived velocity commands")
    args, _ = ap.parse_known_args()

    # Patch the hardware gamepad BEFORE the simulator constructs it in __init__.
    _gp.Se2Gamepad = lambda *a, **k: ZedVelocityPad(
        host=args.host, port=args.port,
        gain_x=args.gain_x, gain_y=args.gain_y, gain_yaw=args.gain_yaw,
        sign_x=args.sign_x, sign_y=args.sign_y, sign_yaw=args.sign_yaw,
        deadzone=args.deadzone, debug=args.debug,
    )

    from berkeley_humanoid_lite.environments import MujocoSimulator, Cfg
    from berkeley_humanoid_lite_lowlevel.policy.rl_controller import RlController

    sys.argv = [sys.argv[0], "--config", args.config]
    cfg = Cfg.from_arguments()
    if not cfg:
        raise ValueError("Failed to load config.")

    print(f"[zed-teleop] config={args.config}  ZED={args.host}:{args.port}")
    robot = MujocoSimulator(cfg)
    obs = robot.reset()

    controller = RlController(cfg)
    controller.load_policy()

    default_actions = np.array(cfg.default_joint_positions, dtype=np.float32)[robot.cfg.action_indices]

    while True:
        actions = controller.update(obs.numpy())
        if actions is None:
            actions = default_actions
        actions = torch.tensor(actions)
        obs = robot.step(actions)


if __name__ == "__main__":
    main()
