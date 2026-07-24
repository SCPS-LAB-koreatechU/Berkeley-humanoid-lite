# WSL-friendly sim2sim launcher for Berkeley Humanoid Lite.
# Same as play_mujoco.py but replaces the USB gamepad with a constant command
# (set via --vx/--vy/--vyaw) so the robot can be driven without hardware.
# The MuJoCo passive viewer still opens via WSLg.
import sys
import argparse

import numpy as np
import torch

# --- replace the hardware gamepad with a CLI-driven constant command ---
import berkeley_humanoid_lite_lowlevel.policy.gamepad as _gp


class _ConstantPad:
    def __init__(self, vx=0.0, vy=0.0, vyaw=0.0):
        # mode_switch=3 => RL control mode (matches A+RB on the real pad)
        self.commands = {
            "velocity_x": float(vx),
            "velocity_y": float(vy),
            "velocity_yaw": float(vyaw),
            "mode_switch": 3,
        }

    def run(self):
        # no background thread; command is static
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--vx", type=float, default=0.0, help="forward velocity command")
    ap.add_argument("--vy", type=float, default=0.0, help="lateral velocity command")
    ap.add_argument("--vyaw", type=float, default=0.0, help="yaw velocity command")
    args, _ = ap.parse_known_args()

    # patch BEFORE importing the simulator (it constructs Se2Gamepad in __init__)
    _gp.Se2Gamepad = lambda *a, **k: _ConstantPad(args.vx, args.vy, args.vyaw)

    from berkeley_humanoid_lite.environments import MujocoSimulator, Cfg
    from berkeley_humanoid_lite_lowlevel.policy.rl_controller import RlController

    # Cfg.from_arguments reads --config from argv
    sys.argv = [sys.argv[0], "--config", args.config]
    cfg = Cfg.from_arguments()
    if not cfg:
        raise ValueError("Failed to load config.")

    print(f"[wsl-launcher] command velocity = ({args.vx}, {args.vy}, {args.vyaw})")

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
