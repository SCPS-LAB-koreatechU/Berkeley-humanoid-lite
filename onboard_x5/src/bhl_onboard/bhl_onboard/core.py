"""The board's control core, free of ROS so it can be tested on its own.

One instance owns every CAN socket. Only the control thread calls step();
everything else (ROS callbacks) talks to it through set_lowcmd(),
set_hand_cmd() and call(), which queues a request that step() executes
between two cycles -- so bus traffic never interleaves.

States (BoardState.STATE_*):
  IDLE     outputs off, every present motor read-only (round-robin SDO reads)
  DAMPING  firmware damping on the motors that were driven; entered on
           command loss or by request
  ACTIVE   enabled motors in firmware POSITION mode, following /lowcmd
  ESTOP    damping, latched until reset_estop
"""

from __future__ import annotations

import concurrent.futures
import math
import queue
import threading
import time
from dataclasses import dataclass, field

from . import recoil
from .config import NUM_HAND_MOTOR, NUM_MOTOR, Config, MotorSpec, save_offsets
from .hand import HandDriver, make_hand

IDLE, DAMPING, ACTIVE, ESTOP = 0, 1, 2, 3
STATE_NAMES = {IDLE: "IDLE", DAMPING: "DAMPING", ACTIVE: "ACTIVE", ESTOP: "ESTOP"}

FAULT_LOST = 1
FAULT_OUT_OF_RANGE = 2
FAULT_BUS = 4
FAULT_FIRMWARE = 8
FAULT_CLAMPED = 16

MODE_ABSENT, MODE_IDLE, MODE_DAMPING, MODE_POSITION = 0, 1, 2, 3


@dataclass
class Motor:
    spec: MotorSpec
    live: bool = False
    damping: bool = False
    seen: bool = False
    q: float = 0.0
    dq: float = 0.0
    raw: float = 0.0
    last_reply: float = 0.0
    target: float = 0.0
    target_dq: float = 0.0
    cmd: float = 0.0
    slewing: bool = False
    missed: int = 0
    poll_missed: int = 0
    poll_sent: float = 0.0
    fault: int = 0
    last_feed: float = 0.0
    last_sdo: float = 0.0
    wanted: dict = field(default_factory=dict)    # param -> value
    applied: dict = field(default_factory=dict)

    @property
    def mode(self) -> int:
        if self.live:
            return MODE_POSITION
        if self.damping:
            return MODE_DAMPING
        return MODE_IDLE if self.seen else MODE_ABSENT


@dataclass
class Channel:
    name: str
    bus: recoil.RecoilBus | None = None
    present: bool = False
    up: bool = False
    ifindex: int | None = None
    controller_state: str = ""
    poll_cursor: int = 0
    sdo_cursor: int = 0
    note: str = ""


@dataclass
class LowCmd:
    stamp: float
    mode: list
    q: list
    dq: list
    tau: list
    kp: list
    kd: list


class Core:
    LINK_CHECK_PERIOD = 0.5
    CONTROLLER_STATE_PERIOD = 1.0

    def __init__(self, config: Config, transport_factory=None, sim: bool = False,
                 log=print) -> None:
        self.config = config
        self.sim = sim
        self.log = log
        c = config.section("control")
        self.rate = float(c.get("rate_hz", 200.0))
        self.period = 1.0 / self.rate
        self.reply_budget = float(c.get("reply_budget", 0.003))
        self.hand_divider = max(1, int(c.get("hand_divider", 4)))
        k = config.section("command")
        self.cmd_timeout = float(k.get("timeout", 0.25))
        self.hold_then_damp = float(k.get("hold_then_damp", 1.0))
        s = config.section("safety")
        self.max_speed = float(s.get("max_joint_speed", 1.0))
        self.margin = float(s.get("limit_margin", 0.05))
        self.enable_slack = math.radians(float(s.get("enable_slack_deg", 3.0)))
        self.flaky = int(s.get("link_flaky_cycles", 5))
        self.loss = int(s.get("pdo_loss_threshold", 100))
        self.heartbeat = float(s.get("heartbeat_period", 0.2))
        self.sdo_min_period = float(s.get("sdo_min_period", 0.2))
        self.caps = {"position_kp": 1.5, "position_kd": 0.05, "torque_limit": 1.5, "tau": 0.5}
        self.caps.update({k2: float(v) for k2, v in (s.get("caps") or {}).items()})
        self.gains = {"position_kp": 0.3, "position_kd": 0.01, "torque_limit": 0.5}
        self.gains.update({k2: float(v) for k2, v in config.section("gains").items()})

        self.motors = [Motor(spec=m) for m in config.motors]
        self.channels = {name: Channel(name) for name in config.channels}
        self.transport_factory = transport_factory
        self.hands: dict[str, HandDriver] = {}
        self.hand_cmd: dict[str, tuple[float, list, list] | None] = {}
        for side, spec in config.hands.items():
            self.hands[side] = make_hand(spec, None, sim=sim)
            self.hand_cmd[side] = None

        self.state = IDLE
        self.tick = 0
        self.lock = threading.Lock()
        self.lowcmd: LowCmd | None = None
        self.enable_time = 0.0
        self.requests: queue.Queue = queue.Queue()
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=max(1, len(self.channels)))
        self.last_link_check = 0.0
        self.last_state_check = 0.0
        self.events: list[str] = []       # recent notable events, for status
        self.command_note = ""
        self.last_follow = 0.0

    # ================================================================ API

    def set_lowcmd(self, cmd: LowCmd) -> None:
        with self.lock:
            self.lowcmd = cmd

    def set_hand_cmd(self, side: str, mode: list, q: list, stamp: float) -> None:
        with self.lock:
            if side in self.hand_cmd:
                self.hand_cmd[side] = (stamp, list(mode), list(q))

    def call(self, name: str, timeout: float = 2.0) -> tuple[bool, str]:
        """Run a request (enable/disable/damping/estop/reset_estop/zero) in the
        control thread and wait for its answer."""
        done = threading.Event()
        box: list = []
        self.requests.put((name, box, done))
        if not done.wait(timeout):
            return False, f"{name}: control loop did not answer within {timeout} s"
        return box[0]

    # ================================================================ cycle

    def step(self) -> None:
        now = time.monotonic()
        self.tick += 1
        self._service_links(now)
        while True:
            try:
                name, box, done = self.requests.get_nowait()
            except queue.Empty:
                break
            try:
                box.append(self._handle_request(name))
            except Exception as error:  # noqa: BLE001 - never kill the loop
                box.append((False, f"{name} failed: {error}"))
            done.set()

        if self.state == ACTIVE:
            self._follow_command(now)
        self._exchange_all(now)
        if self.tick % self.hand_divider == 0:
            self._exchange_hands(now)

    # ------------------------------------------------------------ command

    def _follow_command(self, now: float) -> None:
        with self.lock:
            cmd = self.lowcmd
        if cmd is not None and cmd.stamp >= self.enable_time:
            age = now - cmd.stamp
            if age <= self.cmd_timeout:
                self.command_note = ""
                for m in self.motors:
                    i = m.spec.slot
                    if not m.live or cmd.mode[i] != 1:
                        continue
                    q = cmd.q[i]
                    if not math.isfinite(q):
                        continue
                    lower, upper = m.spec.lower + self.margin, m.spec.upper - self.margin
                    clamped = min(max(q, lower), upper)
                    if clamped != q:
                        m.fault |= FAULT_CLAMPED
                        m.target_dq = 0.0
                    else:
                        m.fault &= ~FAULT_CLAMPED
                        dq = cmd.dq[i]
                        m.target_dq = dq if math.isfinite(dq) else 0.0
                    m.target = clamped
                    self._want_gains(m, cmd.kp[i], cmd.kd[i], cmd.tau[i])
            elif age <= self.cmd_timeout + self.hold_then_damp:
                if not self.command_note:
                    self.command_note = f"/lowcmd silent {age:.2f} s: holding"
                    self._event(self.command_note)
                for m in self.motors:
                    m.target_dq = 0.0
            else:
                self._event(f"/lowcmd silent {age:.2f} s: entering DAMPING")
                self._enter_damping(DAMPING)
                return

        # Slew by elapsed time, not by cycles: a loop running slower than its
        # nominal rate (a busy X5) must still move at max_speed. Capped at two
        # periods so a stall never turns into a jump.
        dt = min(max(now - self.last_follow, 0.0), 2.0 * self.period) if self.last_follow else self.period
        self.last_follow = now
        step = self.max_speed * dt
        for m in self.motors:
            if not m.live or m.missed >= self.flaky:
                continue
            delta = m.target - m.cmd
            if abs(delta) > step:
                m.cmd += math.copysign(step, delta)
                m.slewing = True
            else:
                m.cmd = m.target
                m.slewing = False

    def _want_gains(self, m: Motor, kp: float, kd: float, tau: float) -> None:
        if math.isfinite(kp) and kp > 0:
            m.wanted[recoil.Parameter.POSITION_KP] = min(kp, self.caps["position_kp"])
        if math.isfinite(kd) and kd > 0:
            m.wanted[recoil.Parameter.VELOCITY_KP] = min(kd, self.caps["position_kd"])
        if math.isfinite(tau):
            cap = self.caps["tau"]
            m.wanted[recoil.Parameter.TORQUE_TARGET] = min(max(tau, -cap), cap)

    # ------------------------------------------------------------ buses

    def _service_links(self, now: float) -> None:
        if now - self.last_link_check < self.LINK_CHECK_PERIOD and self.tick > 1:
            return
        self.last_link_check = now
        check_controller = now - self.last_state_check >= self.CONTROLLER_STATE_PERIOD
        if check_controller:
            self.last_state_check = now
        for ch in self.channels.values():
            if self.sim:
                present, up, ifindex = True, True, 1
            else:
                present, up, ifindex = recoil.link_info(ch.name)
            ch.present, ch.up = present, up
            if ch.bus is not None and (not up or ifindex != ch.ifindex):
                self._close_channel(ch, "link down" if not up else "adapter re-plugged")
            if ch.bus is None and up:
                try:
                    transport = self.transport_factory(ch.name)
                except Exception as error:  # noqa: BLE001 - adapter half-gone
                    if ch.note != str(error):
                        ch.note = str(error)
                        self._event(f"{ch.name}: cannot open ({error})")
                    continue
                ch.bus = recoil.RecoilBus(ch.name, transport)
                ch.ifindex = ifindex
                ch.note = ""
                self._event(f"{ch.name}: opened")
                for m in self.motors:
                    if m.spec.channel == ch.name:
                        m.fault &= ~FAULT_BUS
            if check_controller and not self.sim and present:
                ch.controller_state = recoil.controller_state(ch.name)
            elif self.sim:
                ch.controller_state = "SIM"

    def _close_channel(self, ch: Channel, reason: str) -> None:
        if ch.bus is not None:
            ch.bus.close()
        ch.bus = None
        self._event(f"{ch.name}: closed ({reason})")
        for m in self.motors:
            if m.spec.channel == ch.name and m.spec.present:
                if m.live:
                    m.fault |= FAULT_BUS
                m.live = False
                m.damping = False

    def _exchange_all(self, now: float) -> None:
        open_channels = [ch for ch in self.channels.values() if ch.bus is not None]
        if len(open_channels) == 1:
            # One bus (the X5's on-board can0): no thread hand-off, which costs
            # more than the exchange itself on the A55.
            ch = open_channels[0]
            try:
                self._exchange_channel(ch, now)
            except OSError as error:
                self._close_channel(ch, str(error))
            except Exception as error:  # noqa: BLE001
                self._event(f"{ch.name}: exchange error {error!r}")
            return
        futures = {}
        for ch in self.channels.values():
            if ch.bus is not None:
                futures[ch.name] = self.pool.submit(self._exchange_channel, ch, now)
        for name, fut in futures.items():
            try:
                fut.result()
            except OSError as error:
                self._close_channel(self.channels[name], str(error))
            except Exception as error:  # noqa: BLE001
                self._event(f"{name}: exchange error {error!r}")

    def _exchange_channel(self, ch: Channel, now: float) -> None:
        bus = ch.bus
        motors = [m for m in self.motors if m.spec.channel == ch.name and m.spec.present]
        if not motors:
            bus.drain()
            return
        if bus.quiet:
            # Nobody ACKs: no traffic except one ping a second, so the adapter
            # does not drift into ERROR-PASSIVE / BUS-OFF.
            if bus.probe_due():
                bus.ping(motors[ch.poll_cursor % len(motors)].spec.device_id)
                ch.poll_cursor += 1
            bus.collect(time.monotonic() + self.reply_budget)
            for m in motors:
                if m.live:
                    self._count_miss(m, bus)
            return

        live = [m for m in motors if m.live]
        waiting = set()
        cycle_start = time.monotonic()
        for m in live:
            vff = 0.0 if m.slewing else m.target_dq
            if bus.send_pdo2(m.spec.device_id, m.spec.to_actuator(m.cmd),
                             m.spec.to_actuator_velocity(vff)):
                waiting.add(m.spec.device_id)
            if now - m.last_feed >= self.heartbeat:
                if bus.feed(m.spec.device_id):
                    m.last_feed = now

        # at most one gain rewrite per bus per cycle
        pending = [m for m in live
                   if now - m.last_sdo >= self.sdo_min_period
                   and any(abs(m.applied.get(p, math.nan) - v) > 1e-6
                           or p not in m.applied for p, v in m.wanted.items())]
        if pending:
            m = pending[ch.sdo_cursor % len(pending)]
            ch.sdo_cursor += 1
            for p, v in m.wanted.items():
                if p not in m.applied or abs(m.applied[p] - v) > 1e-6:
                    if bus.write_f32(m.spec.device_id, p, v):
                        m.applied[p] = v
                        m.last_sdo = now
                    break

        # read-only motors: one SDO position read per cycle, round robin
        idle = [m for m in motors if not m.live]
        if idle:
            m = idle[ch.poll_cursor % len(idle)]
            ch.poll_cursor += 1
            # a motor that never answered costs bus time; ask it rarely
            if m.seen or m.poll_missed < 3 or ch.poll_cursor % 100 == 0:
                if bus.request_f32(m.spec.device_id, recoil.Parameter.POSITION_MEASURED):
                    m.poll_sent = cycle_start
                    m.poll_missed += 1

        bus.collect(cycle_start + self.reply_budget, waiting if waiting else None)
        for m in live:
            reply = bus.pdo.get(m.spec.device_id)
            if reply is not None and reply[2] >= cycle_start:
                pos, vel, t = reply
                m.raw = pos
                m.q = m.spec.from_actuator(pos)
                m.dq = m.spec.from_actuator_velocity(vel)
                m.missed = 0
                m.last_reply = t
                m.seen = True
            else:
                self._count_miss(m, bus)
        for m in idle:
            reply = bus.sdo.get(m.spec.device_id)
            if reply is not None and reply[1] >= m.poll_sent > 0:
                pos = recoil.sdo_f32(reply[0])
                if math.isfinite(pos):
                    if not m.seen:
                        self._event(f"{m.spec.name}: motor {m.spec.device_id} answering")
                    m.raw = pos
                    m.q = m.spec.from_actuator(pos)
                    m.dq = 0.0
                    m.seen = True
                    m.last_reply = reply[1]
                    m.poll_missed = 0
                    m.fault &= ~FAULT_LOST

    def _count_miss(self, m: Motor, bus: recoil.RecoilBus) -> None:
        m.missed += 1
        if m.missed >= self.loss:
            m.fault |= FAULT_LOST
            m.live = False
            bus.set_mode(m.spec.device_id, recoil.Mode.IDLE)
            self._event(f"{m.spec.name}: no reply for {m.missed} cycles, dropped to IDLE")

    # ------------------------------------------------------------ hands

    def _exchange_hands(self, now: float) -> None:
        dt = self.period * self.hand_divider
        for side, hand in self.hands.items():
            if self.state == ACTIVE and hand.energised:
                with self.lock:
                    cmd = self.hand_cmd.get(side)
                if cmd is not None and cmd[0] >= self.enable_time \
                        and now - cmd[0] <= self.cmd_timeout:
                    _, mode, q = cmd
                    spec = hand.spec
                    step = 3.0 * dt
                    for i in range(NUM_HAND_MOTOR):
                        if mode[i] != 1 or not math.isfinite(q[i]):
                            continue
                        goal = min(max(q[i], spec.lower[i]), spec.upper[i])
                        delta = goal - hand.target[i]
                        hand.target[i] += max(-step, min(step, delta))
            try:
                hand.exchange()
            except Exception as error:  # noqa: BLE001
                self._event(f"hand {side}: {error!r}")

    # ================================================================ requests

    def _handle_request(self, name: str) -> tuple[bool, str]:
        if name == "enable":
            return self._enable()
        if name == "disable":
            self._enter_idle()
            return True, "outputs off (IDLE)"
        if name == "damping":
            if self.state == ESTOP:
                return False, "E-STOP latched"
            self._enter_damping(DAMPING)
            return True, "DAMPING"
        if name == "estop":
            self._enter_damping(ESTOP)
            return True, "E-STOP latched (damping); reset_estop then enable"
        if name == "reset_estop":
            if self.state != ESTOP:
                return True, f"not latched (state {STATE_NAMES[self.state]})"
            self._enter_idle()
            return True, "E-STOP cleared, IDLE"
        if name == "zero":
            return self._zero()
        return False, f"unknown request {name}"

    def _enable(self) -> tuple[bool, str]:
        if self.state == ESTOP:
            return False, "E-STOP latched: call reset_estop first"
        if self.state == ACTIVE:
            return True, "already ACTIVE"
        energised, skipped = [], []
        now = time.monotonic()
        for m in self.motors:
            if not (m.spec.present and m.spec.enabled):
                continue
            ch = self.channels.get(m.spec.channel)
            bus = ch.bus if ch else None
            if bus is None:
                skipped.append(f"{m.spec.name}: {m.spec.channel} not open")
                continue
            if bus.quiet:
                # Frames sent into a bus nobody ACKs are retried by the adapter
                # forever (seen on the bench) -- send nothing at all.
                skipped.append(f"{m.spec.name}: {m.spec.channel} quiet (motor power?)")
                continue
            m.fault &= ~(FAULT_LOST | FAULT_OUT_OF_RANGE | FAULT_BUS)
            if not m.seen or now - m.last_reply > 0.5:
                q = self._read_position_now(bus, m)
                if q is None:
                    m.fault |= FAULT_LOST
                    skipped.append(f"{m.spec.name}: no answer")
                    continue
            if not (m.spec.lower - self.enable_slack <= m.q <= m.spec.upper + self.enable_slack):
                m.fault |= FAULT_OUT_OF_RANGE
                skipped.append(f"{m.spec.name}: at {m.q:+.3f} rad, outside "
                               f"{m.spec.lower:+.2f}..{m.spec.upper:+.2f} (offset?)")
                continue
            gains = {**self.gains, **{k: float(v) for k, v in m.spec.gains.items()}}
            kp = min(gains["position_kp"], self.caps["position_kp"])
            kd = min(gains["position_kd"], self.caps["position_kd"])
            tl = min(gains["torque_limit"], self.caps["torque_limit"])
            dev = m.spec.device_id
            ok = bus.set_mode(dev, recoil.Mode.IDLE)
            ok &= bus.write_f32(dev, recoil.Parameter.POSITION_KP, kp)
            ok &= bus.write_f32(dev, recoil.Parameter.VELOCITY_KP, kd)
            ok &= bus.write_f32(dev, recoil.Parameter.TORQUE_LIMIT, tl)
            ok &= bus.write_f32(dev, recoil.Parameter.TORQUE_TARGET, 0.0)
            ok &= bus.feed(dev)
            if not ok:
                bus.set_mode(dev, recoil.Mode.IDLE)
                skipped.append(f"{m.spec.name}: bus refused frames")
                continue
            m.applied = {recoil.Parameter.POSITION_KP: kp, recoil.Parameter.VELOCITY_KP: kd,
                         recoil.Parameter.TORQUE_TARGET: 0.0}
            m.wanted = dict(m.applied)
            m.target = m.cmd = m.q
            m.target_dq = 0.0
            m.missed = 0
            m.last_feed = now
            m.last_sdo = now
            m.damping = False
            bus.set_mode(dev, recoil.Mode.POSITION)
            m.live = True
            energised.append(m.spec.name)
        if not energised:
            return False, "nothing energised: " + ("; ".join(skipped) or "no enabled motors")
        self.state = ACTIVE
        self.enable_time = time.monotonic()
        self.command_note = ""
        for hand in self.hands.values():
            hand.target = list(hand.q)
            hand.set_energised(True)
        msg = f"ACTIVE: {len(energised)} motors"
        if skipped:
            msg += "; skipped " + "; ".join(skipped)
        self._event(msg)
        return True, msg

    def _read_position_now(self, bus: recoil.RecoilBus, m: Motor, timeout: float = 0.02):
        sent = time.monotonic()
        if not bus.request_f32(m.spec.device_id, recoil.Parameter.POSITION_MEASURED):
            return None
        deadline = sent + timeout
        while time.monotonic() < deadline:
            bus.collect(min(deadline, time.monotonic() + 0.002))
            reply = bus.sdo.get(m.spec.device_id)
            if reply is not None and reply[1] >= sent:
                pos = recoil.sdo_f32(reply[0])
                if not math.isfinite(pos):
                    return None
                m.raw = pos
                m.q = m.spec.from_actuator(pos)
                m.seen = True
                m.last_reply = reply[1]
                return m.q
        return None

    def _for_driven(self):
        for m in self.motors:
            if m.spec.present and m.spec.enabled:
                ch = self.channels.get(m.spec.channel)
                if ch is not None and ch.bus is not None:
                    yield m, ch.bus

    def _enter_damping(self, state: int) -> None:
        for m, bus in self._for_driven():
            if m.live or m.damping or state == ESTOP:
                bus.set_mode(m.spec.device_id, recoil.Mode.DAMPING)
                m.damping = True
            m.live = False
        for hand in self.hands.values():
            hand.set_energised(False)
        if self.state != state:
            self._event(f"state {STATE_NAMES[self.state]} -> {STATE_NAMES[state]}")
        self.state = state

    def _enter_idle(self) -> None:
        for m, bus in self._for_driven():
            if m.live or m.damping:
                bus.set_mode(m.spec.device_id, recoil.Mode.IDLE)
            m.live = False
            m.damping = False
        for hand in self.hands.values():
            hand.set_energised(False)
        if self.state != IDLE:
            self._event(f"state {STATE_NAMES[self.state]} -> IDLE")
        self.state = IDLE

    def _zero(self) -> tuple[bool, str]:
        if self.state != IDLE:
            return False, "zero only in IDLE (outputs off)"
        now = time.monotonic()
        offsets = {}
        for m in self.motors:
            if m.spec.present and m.seen and now - m.last_reply < 0.5:
                m.spec.offset = m.raw / m.spec.gear_ratio * m.spec.direction
                m.q = 0.0
                offsets[m.spec.name] = m.spec.offset
        if not offsets:
            return False, "no motor answered in the last 0.5 s"
        path = save_offsets(offsets)
        return True, f"zeroed {len(offsets)} motors, saved to {path}: " + ", ".join(
            f"{k}={v:+.4f}" for k, v in offsets.items())

    # ================================================================ status

    def _event(self, text: str) -> None:
        self.log(text)
        self.events.append(f"{time.strftime('%H:%M:%S')} {text}")
        del self.events[:-10]

    def faults(self) -> list[str]:
        out = []
        for m in self.motors:
            if m.fault & FAULT_LOST:
                out.append(f"{m.spec.name}: LOST")
            if m.fault & FAULT_OUT_OF_RANGE:
                out.append(f"{m.spec.name}: OUT_OF_RANGE (q {m.q:+.3f})")
            if m.fault & FAULT_BUS:
                out.append(f"{m.spec.name}: BUS")
        for ch in self.channels.values():
            if not ch.present:
                out.append(f"{ch.name}: adapter missing")
            elif not ch.up:
                out.append(f"{ch.name}: link down")
            elif ch.bus is not None and ch.bus.quiet:
                out.append(f"{ch.name}: nothing answers (motor power?)")
        if self.command_note:
            out.append(self.command_note)
        return out

    def shutdown(self) -> None:
        """Leave every driven motor in IDLE, then close the sockets."""
        try:
            self._enter_idle()
            for ch in self.channels.values():
                if ch.bus is not None:
                    ch.bus.drain()
        finally:
            for ch in self.channels.values():
                if ch.bus is not None:
                    ch.bus.close()
                    ch.bus = None
            self.pool.shutdown(wait=False)


def empty_lowcmd(stamp: float = 0.0) -> LowCmd:
    z = [0.0] * NUM_MOTOR
    return LowCmd(stamp, [0] * NUM_MOTOR, list(z), list(z), list(z), list(z), list(z))
