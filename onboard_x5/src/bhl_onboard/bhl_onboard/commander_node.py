"""bhl_commander: runs on the board next to bhl_lowlevel and turns high-level
requests from the PC into the /lowcmd stream (the role unitree's high-level
services play on top of rt/lowcmd).

  action ~controllers~/<name>/follow_joint_trajectory  control_msgs/FollowJointTrajectory
         names from config `commander.controllers` (MoveIt's
         moveit_controllers_hardware.yaml names: arms_controller, ...)
  srv    /bhl/hand_preset  bhl_msgs/HandPreset
  pub    /lowcmd, /hand/{side}/cmd   at commander.rate_hz, while the board is ACTIVE

The trajectory is sampled here, on the board, so Wi-Fi jitter between the PC
and the robot does not reach the motors: the PC sends one goal, the board
streams 200 Hz. When no goal runs, the last target is held.

Only one /lowcmd publisher may exist (bhl_lowlevel refuses commands
otherwise), so stop this node before driving /lowcmd from the PC directly.
"""

from __future__ import annotations

import argparse
import bisect
import functools
import math
import os
import signal
import threading
import time
from dataclasses import dataclass, field

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from bhl_msgs.msg import HandCmd, HandState, LowCmd, LowState
from bhl_msgs.srv import HandPreset

from . import config as config_mod
from .config import NUM_HAND_MOTOR, NUM_MOTOR
from .crc import seal
from .lowlevel_node import COMMAND_QOS, STATE_QOS, default_config_path
from .polled import PolledSubscription, io_node

ACTIVE = 2


def duration_s(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


@dataclass
class Plan:
    """A piecewise cubic Hermite trajectory over a set of joint refs."""
    refs: list                      # ("m", slot) | ("h", side, index)
    times: list
    positions: list                 # [point][joint]
    velocities: list
    start: float
    final_tolerance: float
    done: threading.Event = field(default_factory=threading.Event)
    result: tuple | None = None     # (error_code, text)
    cancelled: bool = False

    @property
    def end(self) -> float:
        return self.start + self.times[-1]

    def sample(self, now: float) -> tuple[list, list, bool]:
        t = now - self.start
        if t >= self.times[-1]:
            return list(self.positions[-1]), [0.0] * len(self.refs), True
        if t <= self.times[0]:
            return list(self.positions[0]), [0.0] * len(self.refs), False
        k = bisect.bisect_right(self.times, t) - 1
        t0, t1 = self.times[k], self.times[k + 1]
        h = t1 - t0
        s = (t - t0) / h
        h00, h10 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s
        h01, h11 = -2 * s**3 + 3 * s**2, s**3 - s**2
        d00, d10 = 6 * s**2 - 6 * s, 3 * s**2 - 4 * s + 1
        d01, d11 = -6 * s**2 + 6 * s, 3 * s**2 - 2 * s
        pos, vel = [], []
        p0, p1 = self.positions[k], self.positions[k + 1]
        v0, v1 = self.velocities[k], self.velocities[k + 1]
        for j in range(len(self.refs)):
            pos.append(h00 * p0[j] + h10 * h * v0[j] + h01 * p1[j] + h11 * h * v1[j])
            vel.append((d00 * p0[j] + d10 * h * v0[j] + d01 * p1[j] + d11 * h * v1[j]) / h)
        return pos, vel, False


class CommanderNode(Node):
    def __init__(self, cfg: config_mod.Config) -> None:
        super().__init__("bhl_commander")
        self.cfg = cfg
        c = cfg.section("commander")
        self.rate = float(c.get("rate_hz", 200.0))
        # hand commands at the board's hand exchange rate, not the arm rate
        control = cfg.section("control")
        hand_rate = float(control.get("rate_hz", 200.0)) / max(1, int(control.get("hand_divider", 4)))
        self.hand_divider = max(1, round(self.rate / hand_rate))
        self.cycle = 0
        self._lowcmd = LowCmd()
        self.goal_time_tolerance = float(c.get("goal_time_tolerance", 0.5))
        self.goal_tolerance = float(c.get("goal_tolerance", 0.05))
        self.unmapped = str(c.get("unmapped_joints", "reject"))
        self.presets = {k: [float(x) for x in v] for k, v in (cfg.raw.get("hand_presets") or {}).items()}
        self.preset_duration = float(cfg.raw.get("hand_preset_duration", 1.0))
        # name -> ref; only joints the board actually drives are commandable.
        # Goals are checked against the URDF limits themselves; the board
        # clamps into them less safety.limit_margin, so a MoveIt plan that
        # touches a limit (elbow pitch home is exactly 0) still runs.
        self.refs: dict[str, tuple] = {}
        self.limits: dict[tuple, tuple[float, float]] = {}
        self.reachable: dict[tuple, tuple[float, float]] = {}   # what the board clamps into
        margin = float(cfg.section("safety").get("limit_margin", 0.05))
        for m in cfg.motors:
            if m.present and m.enabled:
                self.refs[m.name] = ("m", m.slot)
                self.limits[("m", m.slot)] = (m.lower, m.upper)
                self.reachable[("m", m.slot)] = (m.lower + margin, m.upper - margin)
        self.present_slots = [m.slot for m in cfg.motors if m.present and m.enabled]
        for side, h in cfg.hands.items():
            for i, name in enumerate(h.joints):
                self.refs[name] = ("h", side, i)
                self.limits[("h", side, i)] = (h.lower[i], h.upper[i])

        self.lock = threading.Lock()
        self.board_state = -1
        self.measured = [0.0] * NUM_MOTOR
        self.hand_measured = {s: [0.0] * NUM_HAND_MOTOR for s in cfg.hands}
        self.hand_online = {s: False for s in cfg.hands}
        self.hold = [0.0] * NUM_MOTOR
        self.hold_v = [0.0] * NUM_MOTOR
        self.hand_hold = {s: [0.0] * NUM_HAND_MOTOR for s in cfg.hands}
        self.synced = False
        self.plans: dict[str, Plan] = {}     # controller / "hand:<side>" -> plan
        self.last_state_time = 0.0

        cb = ReentrantCallbackGroup()
        self.lowcmd_pub = self.create_publisher(LowCmd, "/lowcmd", STATE_QOS)
        self.hand_pubs = {s: self.create_publisher(HandCmd, f"/hand/{s}/cmd", STATE_QOS)
                          for s in cfg.hands}
        # State inputs are taken by the stream thread (see polled.py): the
        # executor wakes only for goals, cancels and the preset service.
        self.io = io_node("bhl_commander_io")
        self.lowstate_sub = PolledSubscription(self.io, LowState, "/lowstate", COMMAND_QOS)
        self.hand_state_subs = {s: PolledSubscription(self.io, HandState, f"/hand/{s}/state",
                                                      COMMAND_QOS) for s in cfg.hands}
        self._hand_cmds = {s: HandCmd() for s in cfg.hands}
        self.servers = []
        for name, joints in (c.get("controllers") or {}).items():
            self.servers.append(ActionServer(
                self, FollowJointTrajectory, f"/{name}/follow_joint_trajectory",
                # partial keeps it a coroutine function, so rclpy awaits it
                execute_callback=functools.partial(self._execute, name),
                goal_callback=lambda goal, n=name: self._on_goal(n, goal),
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=cb))
            self.get_logger().info(f"action /{name}/follow_joint_trajectory ({len(joints)} joints)")
        self.create_service(HandPreset, "/bhl/hand_preset", self._srv_preset, callback_group=cb)
        # Goal executors are coroutines on a single-threaded executor (the
        # multi-threaded one costs ~3x the CPU, which the X5 cannot spare);
        # they sleep on futures that the stream thread releases ~20 times a
        # second through a guard condition -- no timer waking an idle executor.
        self._sleepers: list[Future] = []
        self._wake = self.create_guard_condition(self._wake_sleepers, callback_group=cb)

        self.running = True
        self.thread = threading.Thread(target=self._stream, name="bhl_commander", daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ state in

    def _on_lowstate(self, msg: LowState) -> None:
        with self.lock:
            self.last_state_time = time.monotonic()
            for i, s in enumerate(msg.motor_state):
                self.measured[i] = s.q
            was_active = self.board_state == ACTIVE
            self.board_state = msg.board_state
            if msg.board_state == ACTIVE and not was_active:
                # Board just came up: hold where the joints are, never a stale target.
                self.hold = list(self.measured)
                self.hold_v = [0.0] * NUM_MOTOR
                for side in self.hand_hold:
                    self.hand_hold[side] = list(self.hand_measured[side])
                self.synced = True
                self.get_logger().info("board ACTIVE: holding the measured pose")
            elif msg.board_state != ACTIVE:
                self.synced = False
                for plan in list(self.plans.values()):
                    self._finish(plan, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                 "board left ACTIVE")
                self.plans.clear()

    def _on_hand_state(self, side: str, msg: HandState) -> None:
        with self.lock:
            self.hand_online[side] = bool(msg.online)
            self.hand_measured[side] = [s.q for s in msg.motor_state]

    def _current(self, ref) -> float:
        if ref[0] == "m":
            return self.hold[ref[1]]
        return self.hand_hold[ref[1]][ref[2]]

    def _measured(self, ref) -> float:
        if ref[0] == "m":
            return self.measured[ref[1]]
        return self.hand_measured[ref[1]][ref[2]]

    # ------------------------------------------------------------ goals

    def _resolve(self, names: list[str]) -> tuple[list, list[int], str]:
        refs, keep, missing = [], [], []
        for j, name in enumerate(names):
            ref = self.refs.get(name)
            if ref is None or (ref[0] == "h" and not self.hand_online.get(ref[1], False)):
                missing.append(name)
                continue
            refs.append(ref)
            keep.append(j)
        return refs, keep, ", ".join(missing)

    def _on_goal(self, controller: str, goal) -> GoalResponse:
        traj = goal.trajectory
        if not traj.points:
            return GoalResponse.REJECT
        refs, _, missing = self._resolve(list(traj.joint_names))
        if missing and self.unmapped == "reject":
            self.get_logger().error(f"{controller}: rejected, not driven by the board: {missing}")
            return GoalResponse.REJECT
        if not refs:
            self.get_logger().error(f"{controller}: rejected, none of its joints is driven")
            return GoalResponse.REJECT
        with self.lock:
            if self.board_state != ACTIVE or not self.synced:
                self.get_logger().error(f"{controller}: rejected, board is not ACTIVE "
                                        "(call /bhl_lowlevel/enable)")
                return GoalResponse.REJECT
        if missing:
            self.get_logger().warn(f"{controller}: ignoring joints not driven by the board: {missing}")
        return GoalResponse.ACCEPT

    def _make_plan(self, traj) -> Plan | str:
        refs, keep, _ = self._resolve(list(traj.joint_names))
        times, positions, velocities = [], [], []
        for p in traj.points:
            if len(p.positions) != len(traj.joint_names):
                return "a point has the wrong number of positions"
            t = duration_s(p.time_from_start)
            if times and t <= times[-1]:
                return "time_from_start must increase"
            times.append(t)
            pos = [p.positions[j] for j in keep]
            vel = [p.velocities[j] for j in keep] if len(p.velocities) == len(traj.joint_names) \
                else None
            if not all(math.isfinite(x) for x in pos):
                return "non-finite position"
            for ref, x in zip(refs, pos):
                lo, hi = self.limits[ref]
                if not lo - 1e-3 <= x <= hi + 1e-3:
                    return f"point at {t:.2f}s outside limits for {ref}: {x:+.3f} not in {lo:+.3f}..{hi:+.3f}"
            positions.append(pos)
            velocities.append(vel)
        # missing velocities: finite differences (zero at the ends)
        for k in range(len(times)):
            if velocities[k] is None:
                if k == 0 or k == len(times) - 1:
                    velocities[k] = [0.0] * len(refs)
                else:
                    dt = times[k + 1] - times[k - 1]
                    velocities[k] = [(positions[k + 1][j] - positions[k - 1][j]) / dt
                                     for j in range(len(refs))]
        with self.lock:
            start_pos = [self._current(r) for r in refs]
        if times[0] > 0.0:
            times.insert(0, 0.0)
            positions.insert(0, start_pos)
            velocities.insert(0, [0.0] * len(refs))
        else:
            positions[0] = start_pos    # start exactly where we are holding
        return Plan(refs=refs, times=times, positions=positions, velocities=velocities,
                    start=time.monotonic(), final_tolerance=self.goal_tolerance)

    def _finish(self, plan: Plan, code: int, text: str) -> None:
        if not plan.done.is_set():
            plan.result = (code, text)
            plan.done.set()

    def _wake_sleepers(self) -> None:
        sleepers, self._sleepers = self._sleepers, []
        for future in sleepers:
            future.set_result(None)

    def _tick(self) -> Future:
        future = Future()
        self._sleepers.append(future)
        return future

    async def _execute(self, controller: str, goal_handle):
        result = FollowJointTrajectory.Result()
        plan = self._make_plan(goal_handle.request.trajectory)
        if isinstance(plan, str):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = plan
            goal_handle.abort()
            return result
        key = controller
        with self.lock:
            # A new goal on a joint already moving pre-empts the old one.
            for other_key, other in list(self.plans.items()):
                if set(other.refs) & set(plan.refs):
                    self._finish(other, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                 "pre-empted by a newer goal")
                    del self.plans[other_key]
            self.plans[key] = plan
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = [n for n in goal_handle.request.trajectory.joint_names
                                if n in self.refs]
        while not plan.done.is_set():
            await self._tick()
            if plan.done.is_set():
                break
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self._freeze(plan)
                    self.plans.pop(key, None)
                goal_handle.canceled()
                result.error_string = "cancelled; holding"
                return result
            now = time.monotonic()
            with self.lock:
                if now > plan.end:
                    err = max(abs(self._measured(r) - self._expected(r, p))
                              for r, p in zip(plan.refs, plan.positions[-1]))
                    if err <= plan.final_tolerance:
                        self._finish(plan, FollowJointTrajectory.Result.SUCCESSFUL, "")
                    elif now > plan.end + self.goal_time_tolerance:
                        self._finish(plan, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                     f"final error {err:.3f} rad > {plan.final_tolerance}")
            feedback.header.stamp = self.get_clock().now().to_msg()
            goal_handle.publish_feedback(feedback)
        with self.lock:
            if self.plans.get(key) is plan:
                del self.plans[key]
        code, text = plan.result
        result.error_code, result.error_string = code, text
        if code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            self.get_logger().warn(f"{controller}: {text}")
            goal_handle.abort()
        return result

    def _expected(self, ref, goal: float) -> float:
        """Where the board will actually put the joint: a goal on a URDF limit
        ends limit_margin inside it."""
        lo, hi = self.reachable.get(ref, (-math.inf, math.inf))
        return min(max(goal, lo), hi)

    def _freeze(self, plan: Plan) -> None:
        """Hold the measured position of a stopped plan's joints."""
        for r in plan.refs:
            if r[0] == "m":
                self.hold[r[1]] = self.measured[r[1]]
                self.hold_v[r[1]] = 0.0
            else:
                self.hand_hold[r[1]][r[2]] = self.hand_measured[r[1]][r[2]]

    def _srv_preset(self, request, response):
        values = self.presets.get(request.name)
        if values is None:
            response.success = False
            response.message = f"unknown preset '{request.name}' (have: {', '.join(self.presets)})"
            return response
        sides = list(self.cfg.hands) if request.side == "both" else [request.side]
        duration = request.duration if request.duration > 0 else self.preset_duration
        started = []
        with self.lock:
            if self.board_state != ACTIVE or not self.synced:
                response.success, response.message = False, "board is not ACTIVE"
                return response
            for side in sides:
                if side not in self.cfg.hands:
                    response.success, response.message = False, f"no hand '{side}'"
                    return response
                if not self.hand_online.get(side):
                    continue
                refs = [("h", side, i) for i in range(NUM_HAND_MOTOR)]
                start = [self.hand_hold[side][i] for i in range(NUM_HAND_MOTOR)]
                goal = [min(max(v, self.limits[r][0]), self.limits[r][1]) for r, v in zip(refs, values)]
                key = f"hand:{side}"
                if key in self.plans:
                    self._finish(self.plans[key], FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                 "pre-empted by a preset")
                zero = [0.0] * NUM_HAND_MOTOR
                self.plans[key] = Plan(refs=refs, times=[0.0, duration], positions=[start, goal],
                                       velocities=[zero, zero], start=time.monotonic(),
                                       final_tolerance=math.inf)
                started.append(side)
        response.success = bool(started)
        response.message = (f"{request.name} on {', '.join(started)} over {duration:.1f} s"
                            if started else "no hand online")
        return response

    # ------------------------------------------------------------ stream

    def _stream(self) -> None:
        period = 1.0 / self.rate
        next_tick = time.perf_counter()
        while self.running:
            try:
                self._publish_once()
            except Exception as error:  # noqa: BLE001
                if time.monotonic() - getattr(self, "_last_error", 0.0) >= 1.0:
                    self._last_error = time.monotonic()
                    self.get_logger().error(f"stream failed: {error!r}")
            next_tick += period
            delay = next_tick - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -5 * period:
                next_tick = time.perf_counter()

    def _take_inputs(self) -> None:
        msg = self.lowstate_sub.take_latest()
        if msg is not None:
            self._on_lowstate(msg)
        for side, sub in self.hand_state_subs.items():
            msg = sub.take_latest()
            if msg is not None:
                self._on_hand_state(side, msg)

    def _publish_once(self) -> None:
        self._take_inputs()
        if self._sleepers and self.cycle % max(1, round(self.rate / 20)) == 0:
            self._wake.trigger()
        now = time.monotonic()
        with self.lock:
            active = self.board_state == ACTIVE and self.synced \
                and now - self.last_state_time < 0.5
            if active:
                for key, plan in list(self.plans.items()):
                    pos, vel, finished = plan.sample(now)
                    for r, p, v in zip(plan.refs, pos, vel):
                        if r[0] == "m":
                            self.hold[r[1]], self.hold_v[r[1]] = p, v
                        else:
                            self.hand_hold[r[1]][r[2]] = p
                    if finished and key.startswith("hand:"):
                        self._finish(plan, FollowJointTrajectory.Result.SUCCESSFUL, "")
                        del self.plans[key]
            hold, hold_v = list(self.hold), list(self.hold_v)
            hands = {s: list(v) for s, v in self.hand_hold.items()}

        msg = self._lowcmd               # reused: building LowCmd is costly on the A55
        for i in self.present_slots:
            c = msg.motor_cmd[i]
            c.mode = 1 if active else 0
            c.q = float(hold[i]) if active else 0.0
            c.dq = float(hold_v[i]) if active else 0.0
        self.lowcmd_pub.publish(seal(msg))
        self.cycle += 1
        if self.cycle % self.hand_divider:
            return
        for side, pub in self.hand_pubs.items():
            hc = self._hand_cmds[side]
            on = active and self.hand_online.get(side)
            for i, c in enumerate(hc.motor_cmd):
                c.mode = 1 if on else 0
                c.q = float(hands[side][i]) if on else 0.0
            pub.publish(hc)

    def shutdown(self) -> None:
        self.running = False
        self.thread.join(timeout=1.0)
        self.io.destroy_node()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--config", default=os.environ.get("BHL_CONFIG") or default_config_path())
    args, ros_args = parser.parse_known_args(argv)
    ros_args = [a for a in ros_args if a]
    rclpy.init(args=ros_args)
    node = CommanderNode(config_mod.load(args.config))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        stop.wait()
    finally:
        node.shutdown()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
