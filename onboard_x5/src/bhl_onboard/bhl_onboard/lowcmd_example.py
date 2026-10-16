"""Direct /lowcmd client, the counterpart of unitree_sdk2's
example/g1/low_level/g1_dual_arm_example.cpp. Runs on the PC (or the board).

  1. wait for /lowstate and the board to be ACTIVE
  2. ramp from the measured pose to `--pose` over --ramp seconds
  3. sine on one joint (--joint, --amplitude, --frequency) for --duration s
  4. ramp back to where it started, then stop publishing (the board holds,
     and after command.hold_then_damp it damps)

bhl_commander also publishes /lowcmd; stop it first (the board ignores
/lowcmd while two publishers exist).

  ros2 run bhl_onboard bhl_lowcmd_example --joint arm_left_shoulder_yaw_joint --amplitude 0.2
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node

from bhl_msgs.msg import LowCmd, LowState

from . import config as config_mod
from .crc import seal
from .lowlevel_node import COMMAND_QOS, STATE_QOS, default_config_path

ACTIVE = 2


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="unitree-style /lowcmd example")
    parser.add_argument("--config", default=default_config_path())
    parser.add_argument("--joint", default="arm_left_shoulder_yaw_joint")
    parser.add_argument("--amplitude", type=float, default=0.15, help="[rad]")
    parser.add_argument("--frequency", type=float, default=0.3, help="[Hz]")
    parser.add_argument("--duration", type=float, default=10.0, help="[s]")
    parser.add_argument("--ramp", type=float, default=2.0, help="[s]")
    parser.add_argument("--rate", type=float, default=100.0, help="[Hz]")
    args, ros_args = parser.parse_known_args(argv)

    cfg = config_mod.load(args.config, offsets_file=None)
    names = {m.name: m for m in cfg.motors if m.present and m.enabled}
    if args.joint not in names:
        raise SystemExit(f"{args.joint} is not an enabled motor; choose from {sorted(names)}")
    spec = names[args.joint]
    if abs(args.amplitude) > 0.5:
        raise SystemExit("--amplitude above 0.5 rad refused")

    rclpy.init(args=ros_args)
    node = Node("bhl_lowcmd_example")
    pub = node.create_publisher(LowCmd, "/lowcmd", STATE_QOS)
    latest: list = []
    def on_state(msg):
        latest[:] = [msg]
    node.create_subscription(LowState, "/lowstate", on_state, COMMAND_QOS)

    log = node.get_logger()
    log.info("waiting for /lowstate ...")
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and not latest and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not latest:
        raise SystemExit("no /lowstate: is bhl_lowlevel running and ROS_DOMAIN_ID the same?")
    if latest[0].board_state != ACTIVE:
        raise SystemExit("board is not ACTIVE: ros2 service call /bhl_lowlevel/enable "
                         "std_srvs/srv/SetBool '{data: true}'")
    if node.count_publishers("/lowcmd") > 1:
        raise SystemExit("another /lowcmd publisher exists (bhl_commander?) -- stop it first")

    start = [s.q for s in latest[0].motor_state]
    centre = min(max(start[spec.slot], spec.lower + abs(args.amplitude) + 0.05),
                 spec.upper - abs(args.amplitude) - 0.05)
    slots = [m.slot for m in names.values()]
    period = 1.0 / args.rate
    t0 = time.monotonic()
    total = 2 * args.ramp + args.duration
    low, high = start[spec.slot], start[spec.slot]
    log.info(f"{args.joint}: centre {centre:+.3f}, +-{args.amplitude} rad at {args.frequency} Hz")
    try:
        while rclpy.ok():
            t = time.monotonic() - t0
            if t > total:
                break
            target = list(start)
            if t < args.ramp:
                a = 0.5 - 0.5 * math.cos(math.pi * t / args.ramp)
                target[spec.slot] = start[spec.slot] + (centre - start[spec.slot]) * a
            elif t < args.ramp + args.duration:
                target[spec.slot] = centre + args.amplitude * math.sin(
                    2 * math.pi * args.frequency * (t - args.ramp))
            else:
                a = 0.5 - 0.5 * math.cos(math.pi * (t - args.ramp - args.duration) / args.ramp)
                end = centre + args.amplitude * math.sin(2 * math.pi * args.frequency * args.duration)
                target[spec.slot] = end + (start[spec.slot] - end) * a
            msg = LowCmd()
            for i in slots:
                msg.motor_cmd[i].mode = 1
                msg.motor_cmd[i].q = float(target[i])
            pub.publish(seal(msg))
            rclpy.spin_once(node, timeout_sec=0.0)
            q_now = latest[0].motor_state[spec.slot].q
            low, high = min(low, q_now), max(high, q_now)
            time.sleep(max(0.0, period - ((time.monotonic() - t0) % period)))
        q = latest[0].motor_state[spec.slot].q
        log.info(f"done; {args.joint} moved over {low:+.3f}..{high:+.3f} rad, now {q:+.3f} "
                 f"(started {start[spec.slot]:+.3f}). The board holds, then damps after "
                 "command.hold_then_damp unless something else streams /lowcmd.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
