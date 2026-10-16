"""A python-can-shaped transport with simulated recoil motors behind it.

Used by `sim:=true` and the tests. Each motor behaves like the firmware as far
as the board relies on it: NMT mode changes, SDO read/write of the parameters
in recoil.Parameter, PDO_2 position exchange (it only moves in POSITION mode,
first-order towards the target), ping, heartbeat. Faults can be injected:
`powered=False` (nobody ACKs -> no replies at all), per-motor `present=False`,
`drop_rate`.
"""

from __future__ import annotations

import collections
import math
import random
import struct
import threading
import time

import can

from .recoil import Function, Mode, Parameter, make_id, split_id


class FakeMotor:
    def __init__(self, device_id: int, position: float = 0.0) -> None:
        self.device_id = device_id
        self.present = True
        self.mode = Mode.IDLE
        self.position = position      # rotor rad (firmware gear_ratio 1)
        self.velocity = 0.0
        self.target = position
        self.target_velocity = 0.0
        self.params = {Parameter.POSITION_KP: 0.2, Parameter.VELOCITY_KP: 0.005,
                       Parameter.TORQUE_LIMIT: 0.8, Parameter.TORQUE_TARGET: 0.0,
                       Parameter.BUS_VOLTAGE: 24.0, Parameter.ERROR: 0.0}
        self.time_constant = 0.03
        self.stamp = time.monotonic()
        self.mode_history: list[int] = []
        self.writes: list[tuple[int, float]] = []

    def advance(self) -> None:
        now = time.monotonic()
        dt = min(now - self.stamp, 0.1)
        self.stamp = now
        if self.mode == Mode.POSITION:
            alpha = 1.0 - math.exp(-dt / self.time_constant)
            new = self.position + (self.target - self.position) * alpha
            self.velocity = (new - self.position) / dt if dt > 0 else 0.0
            self.position = new
        elif self.mode == Mode.DAMPING:
            self.velocity *= 0.5
        else:
            self.velocity = 0.0


class FakeCanBus:
    def __init__(self, channel: str, motors: dict[int, FakeMotor] | None = None) -> None:
        self.channel = channel
        self.motors = motors or {}
        self.powered = True
        self.drop_rate = 0.0
        self.rx: collections.deque = collections.deque()
        self.cv = threading.Condition()
        self.sent: list[can.Message] = []      # everything the board sent (tests)
        self.keep_sent = False
        self.extra_responder = None            # callable(msg) -> list[can.Message]
        self.closed = False

    # python-can surface ------------------------------------------------

    def send(self, msg: can.Message, timeout=None) -> None:
        if self.closed:
            raise can.CanOperationError("Network is down")
        if self.keep_sent:
            self.sent.append(msg)
        if not self.powered:
            return
        replies = self._handle(msg)
        if self.extra_responder is not None:
            replies += self.extra_responder(msg) or []
        if replies:
            with self.cv:
                for reply in replies:
                    if self.drop_rate and random.random() < self.drop_rate:
                        continue
                    self.rx.append(reply)
                self.cv.notify_all()

    def recv(self, timeout=None):
        with self.cv:
            if not self.rx and timeout:
                self.cv.wait(timeout)
            if self.rx:
                return self.rx.popleft()
        return None

    def shutdown(self) -> None:
        self.closed = True

    # firmware -----------------------------------------------------------

    def _reply(self, function: int, device_id: int, data: bytes) -> can.Message:
        return can.Message(arbitration_id=make_id(function, device_id),
                           is_extended_id=False, data=data, timestamp=time.time())

    def _handle(self, msg: can.Message) -> list[can.Message]:
        function, device_id = split_id(msg.arbitration_id)
        motor = self.motors.get(device_id)
        if motor is None or not motor.present:
            return []
        motor.advance()
        data = bytes(msg.data)
        if function == Function.NMT and len(data) >= 1:
            motor.mode = data[0]
            motor.mode_history.append(data[0])
            if data[0] == Mode.POSITION:
                motor.target = motor.position
            return []
        if function == Function.HEARTBEAT:
            return []
        if function == Function.RECEIVE_PDO_1 and data[:1] == b"\xCA":
            return [self._reply(Function.TRANSMIT_PDO_1, device_id, b"\xCA" + b"\0" * 7)]
        if function == Function.RECEIVE_PDO_2 and len(data) >= 8:
            target, velocity = struct.unpack("<ff", data[:8])
            if motor.mode == Mode.POSITION:
                motor.target, motor.target_velocity = target, velocity
            return [self._reply(Function.TRANSMIT_PDO_2, device_id,
                                struct.pack("<ff", motor.position, motor.velocity))]
        if function == Function.RECEIVE_SDO and len(data) >= 3:
            op = data[0] >> 5
            param = struct.unpack("<H", data[1:3])[0]
            if op == 0x02:
                if param == Parameter.POSITION_MEASURED:
                    value = motor.position
                elif param == Parameter.MODE:
                    return [self._reply(Function.TRANSMIT_SDO, device_id,
                                        struct.pack("<I", motor.mode))]
                else:
                    value = motor.params.get(param, 0.0)
                return [self._reply(Function.TRANSMIT_SDO, device_id, struct.pack("<f", value))]
            if op == 0x01 and len(data) >= 8:
                value = struct.unpack("<f", data[4:8])[0]
                motor.params[param] = value
                motor.writes.append((param, value))
            return []
        return []


def sim_factory(cfg):
    """Transport factory for Core: fake channels holding every present motor
    of `cfg`, each starting at URDF zero (mid-range if zero is outside its
    limits). Re-opening a channel returns the same simulated motors."""
    buses: dict[str, FakeCanBus] = {}

    def factory(channel: str):
        if channel in buses and buses[channel].closed:
            old = buses[channel]
            buses[channel] = FakeCanBus(channel, old.motors)
        if channel not in buses:
            motors = {}
            for m in cfg.motors:
                if m.present and m.channel == channel:
                    q0 = 0.0 if m.lower <= 0.0 <= m.upper else 0.5 * (m.lower + m.upper)
                    motors[m.device_id] = FakeMotor(m.device_id, m.to_actuator(q0))
            buses[channel] = FakeCanBus(channel, motors)
        return buses[channel]

    factory.buses = buses
    return factory
