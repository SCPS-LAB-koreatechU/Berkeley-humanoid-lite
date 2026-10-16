"""bhl_lowlevel: the board's motor service (the role of unitree's low-level
service behind rt/lowcmd / rt/lowstate).

Topics (ROS 2 puts DDS topic "rt/lowcmd" behind ROS name /lowcmd, exactly as
unitree_ros2 does):
  sub /lowcmd                bhl_msgs/LowCmd      one commander at a time
  pub /lowstate              bhl_msgs/LowState
  sub /hand/{left,right}/cmd bhl_msgs/HandCmd
  pub /hand/{left,right}/state bhl_msgs/HandState
  pub /bhl/joint_states      sensor_msgs/JointState, board motors only; on the PC
                             joint_state_publisher merges it into the full
                             URDF /joint_states (demo.launch.py onboard:=true)
  pub /bhl/board_state       bhl_msgs/BoardState  (2 Hz)
Services (std_srvs):
  ~/enable SetBool   true: ACTIVE from the measured pose; false: IDLE
  ~/damping Trigger  ~/estop Trigger  ~/reset_estop Trigger  ~/zero Trigger
"""

from __future__ import annotations

import argparse
import os
import signal
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from bhl_msgs.msg import BoardState, BusState, HandCmd, HandState, LowCmd, LowState

from . import config as config_mod
from . import core as core_mod
from .crc import lowcmd_crc
from .fake_can import sim_factory
from .polled import PolledSubscription, io_node
from .recoil import open_socketcan

# Subscriptions best-effort (accept reliable and best-effort publishers alike),
# publications reliable depth 1 (readable by either kind of subscriber).
COMMAND_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)


def default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("bhl_onboard"), "config", "bhl_x5.yaml")
    except Exception:  # noqa: BLE001 - running from the source tree
        return os.path.join(os.path.dirname(__file__), "..", "config", "bhl_x5.yaml")


class LowLevelNode(Node):
    def __init__(self, cfg: config_mod.Config, sim: bool) -> None:
        super().__init__("bhl_lowlevel")
        self.cfg = cfg
        self.sim = sim
        factory = sim_factory(cfg) if sim else open_socketcan
        self.factory = factory
        self.core = core_mod.Core(cfg, factory, sim=sim, log=self._log)
        control = cfg.section("control")
        self.state_divider = max(1, int(control.get("state_divider", 2)))
        self.require_crc = bool(cfg.section("command").get("require_crc", True))
        self.rejected_crc = 0
        self.rejected_other = 0
        self.command_source = "none"
        self.command_blocked = ""
        self.last_cmd_stamp = 0.0
        self.loop_rate = 0.0
        self.overrun_max = 0.0

        cb_srv = ReentrantCallbackGroup()
        # High-rate inputs are taken by the control thread (see polled.py), so
        # the executor only wakes for services and the 2 Hz status timer.
        self.io = io_node("bhl_lowlevel_io")
        self.lowstate_pub = self.create_publisher(LowState, "/lowstate", STATE_QOS)
        self.joint_pub = self.create_publisher(
            JointState, str(control.get("joint_states_topic", "/bhl/joint_states")), 10)
        self.board_pub = self.create_publisher(BoardState, "/bhl/board_state", 10)
        self.lowcmd_sub = PolledSubscription(self.io, LowCmd, "/lowcmd", COMMAND_QOS)
        self.hand_subs = {}
        self.hand_pubs = {}
        for side in cfg.hands:
            self.hand_pubs[side] = self.create_publisher(HandState, f"/hand/{side}/state", STATE_QOS)
            self.hand_subs[side] = PolledSubscription(self.io, HandCmd, f"/hand/{side}/cmd",
                                                      COMMAND_QOS)
        self.create_service(SetBool, "~/enable", self._srv_enable, callback_group=cb_srv)
        for name in ("damping", "estop", "reset_estop", "zero"):
            self.create_service(Trigger, f"~/{name}",
                                lambda req, res, n=name: self._srv_trigger(n, res),
                                callback_group=cb_srv)
        self.create_timer(0.5, self._publish_board_state, callback_group=cb_srv)

        self._lowstate = LowState()
        self._handstate = {side: HandState() for side in cfg.hands}
        self.joint_names = [m.name for m in cfg.motors if m.present]
        self.joint_slots = [m.slot for m in cfg.motors if m.present]
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="bhl_control", daemon=True)
        self.thread.start()
        mode = "SIM (fake CAN)" if sim else "LIVE"
        self.get_logger().info(
            f"{mode}: {len(self.joint_names)} motors on {', '.join(cfg.channels) or 'no bus'}, "
            f"{self.core.rate:.0f} Hz, config {cfg.path}")

    # ------------------------------------------------------------ logging

    def _log(self, text: str) -> None:
        self.get_logger().info(text)

    # ------------------------------------------------------------ inputs

    def _on_lowcmd(self, msg: LowCmd) -> None:
        if self.require_crc and msg.crc != lowcmd_crc(msg):
            self.rejected_crc += 1
            return
        if self.command_blocked:
            self.rejected_other += 1
            return
        c = msg.motor_cmd
        stamp = time.monotonic()
        self.last_cmd_stamp = stamp
        self.core.set_lowcmd(core_mod.LowCmd(
            stamp=stamp,
            mode=[x.mode for x in c], q=[x.q for x in c], dq=[x.dq for x in c],
            tau=[x.tau for x in c], kp=[x.kp for x in c], kd=[x.kd for x in c]))

    def _on_hand_cmd(self, side: str, msg: HandCmd) -> None:
        if self.command_blocked:
            self.rejected_other += 1
            return
        self.core.set_hand_cmd(side, [x.mode for x in msg.motor_cmd],
                               [x.q for x in msg.motor_cmd], time.monotonic())

    def _take_inputs(self) -> None:
        msg = self.lowcmd_sub.take_latest()
        if msg is not None:
            self._on_lowcmd(msg)
        if self.core.tick % self.core.hand_divider == 0:
            for side, sub in self.hand_subs.items():
                msg = sub.take_latest()
                if msg is not None:
                    self._on_hand_cmd(side, msg)

    def _srv_enable(self, request, response):
        if request.data and self.command_blocked:
            response.success, response.message = False, self.command_blocked
            return response
        response.success, response.message = self.core.call("enable" if request.data else "disable")
        return response

    def _srv_trigger(self, name: str, response):
        response.success, response.message = self.core.call(name)
        return response

    # ------------------------------------------------------------ loop

    def _loop(self) -> None:
        period = self.core.period
        next_tick = time.perf_counter()
        count, window_start, worst = 0, time.perf_counter(), 0.0
        failures, last_report = 0, 0.0
        while self.running:
            try:
                self._take_inputs()
                self.core.step()
                if self.core.tick % self.state_divider == 0:
                    self._publish_state()
            except Exception as error:  # noqa: BLE001 - keep the loop alive, say why
                failures += 1
                if time.monotonic() - last_report >= 1.0:   # one line a second, not 200
                    last_report = time.monotonic()
                    self.get_logger().error(f"control step failed ({failures}x): {error!r}")
                    failures = 0
            count += 1
            next_tick += period
            now = time.perf_counter()
            late = now - next_tick
            if late > 0:
                worst = max(worst, late)
                if late > 5 * period:      # far behind: resync instead of bursting
                    next_tick = now
            else:
                time.sleep(-late)
            if now - window_start >= 1.0:
                self.loop_rate = count / (now - window_start)
                self.overrun_max = worst * 1000.0
                count, window_start, worst = 0, now, 0.0

    # ------------------------------------------------------------ outputs

    def _publish_state(self) -> None:
        # Messages are built once and refilled: constructing LowState (22
        # nested MotorState) every time cost ~14 ms per publish on the X5's A55.
        # publish() serialises synchronously, so reuse is safe.
        core = self.core
        msg = self._lowstate
        msg.tick = core.tick & 0xFFFFFFFF
        msg.board_state = core.state
        for m, out in zip(core.motors, msg.motor_state):
            out.mode = m.mode if m.spec.present else 0
            out.q = float(m.q)
            out.dq = float(m.dq)
            out.error = m.fault
            out.lost = min(m.missed, 0xFFFFFFFF)
        self.lowstate_pub.publish(msg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        names = list(self.joint_names)
        position = [float(core.motors[i].q) for i in self.joint_slots]
        velocity = [float(core.motors[i].dq) for i in self.joint_slots]
        for side, hand in core.hands.items():
            if hand.online:
                names += hand.spec.joints
                position += [float(v) for v in hand.q]
                velocity += [float(v) for v in hand.dq]
            hs = self._handstate[side]
            hs.online = 1 if hand.online else 0
            for i, out in enumerate(hs.motor_state):
                out.mode = (3 if hand.energised else 1) if hand.online else 0
                out.q = float(hand.q[i])
                out.dq = float(hand.dq[i])
            self.hand_pubs[side].publish(hs)
        js.name, js.position, js.velocity = names, position, velocity
        self.joint_pub.publish(js)

    def _publish_board_state(self) -> None:
        # One commander at a time, as with unitree's low-level mode: a second
        # /lowcmd publisher means two programs think they drive the robot.
        infos = self.get_publishers_info_by_topic("/lowcmd")
        names = sorted({f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in infos})
        if len(names) > 1:
            if not self.command_blocked:
                self.get_logger().error(
                    f"/lowcmd has {len(names)} publishers {names}: ignoring all commands "
                    "(the board holds, then damps) until only one is left")
            self.command_blocked = f"multiple /lowcmd publishers: {', '.join(names)}"
        elif self.command_blocked:
            self.get_logger().info("single /lowcmd publisher again; accepting commands")
            self.command_blocked = ""
        self.command_source = names[0] if names else "none"

        core = self.core
        msg = BoardState()
        msg.state = core.state
        msg.state_name = core_mod.STATE_NAMES[core.state]
        msg.command_source = self.command_source
        msg.command_age = float(time.monotonic() - self.last_cmd_stamp) if self.last_cmd_stamp else -1.0
        msg.rejected_crc = self.rejected_crc
        msg.rejected_other = self.rejected_other
        msg.loop_rate = float(self.loop_rate)
        msg.loop_overrun_max = float(self.overrun_max)
        for ch in core.channels.values():
            b = BusState()
            b.name = ch.name
            b.present, b.up, b.open = ch.present, ch.up, ch.bus is not None
            b.controller_state = ch.controller_state
            if ch.bus is not None:
                b.quiet = ch.bus.quiet
                b.tx_dropped = ch.bus.tx_dropped
                b.error_frames = ch.bus.error_frames
            msg.buses.append(b)
        msg.faults = core.faults() + ([self.command_blocked] if self.command_blocked else [])
        self.board_pub.publish(msg)

    def shutdown(self) -> None:
        self.running = False
        self.thread.join(timeout=1.0)
        self.core.shutdown()
        self.io.destroy_node()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--config", default=os.environ.get("BHL_CONFIG") or default_config_path())
    parser.add_argument("--sim", action="store_true", help="fake CAN buses with simulated motors")
    args, ros_args = parser.parse_known_args(argv)
    ros_args = [a for a in ros_args if a]
    rclpy.init(args=ros_args)
    cfg = config_mod.load(args.config)
    node = LowLevelNode(cfg, sim=args.sim)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        stop.wait()
    finally:
        node.get_logger().info("shutting down: motors -> IDLE")
        node.shutdown()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
