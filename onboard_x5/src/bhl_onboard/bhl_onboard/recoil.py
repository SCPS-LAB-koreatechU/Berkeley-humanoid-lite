"""Recoil motor-controller CAN protocol, batched for a fixed-rate loop.

Frame layout and function codes are the ones in
berkeley_humanoid_lite_lowlevel/recoil/core.py (firmware 0x20250226):
11-bit id = function (4 bits) << 7 | device id (7 bits).

The upstream Bus does one blocking transmit/receive per motor. Here every
motor on a bus gets its position frame first and the replies are collected
afterwards, so one cycle costs one round trip per bus instead of one per motor.
Bus access is single-threaded per channel: only the control loop calls in.
"""

from __future__ import annotations

import errno
import struct
import subprocess
import json
import time

import can


class Function:
    NMT = 0b0000
    TIME = 0b0010
    TRANSMIT_PDO_1 = 0b0011
    RECEIVE_PDO_1 = 0b0100
    TRANSMIT_PDO_2 = 0b0101
    RECEIVE_PDO_2 = 0b0110
    TRANSMIT_SDO = 0b1011
    RECEIVE_SDO = 0b1100
    FLASH = 0b1101
    HEARTBEAT = 0b1110


class Mode:
    DISABLED = 0x00
    IDLE = 0x01
    DAMPING = 0x02
    CALIBRATION = 0x05
    CURRENT = 0x10
    TORQUE = 0x11
    VELOCITY = 0x12
    POSITION = 0x13


class Parameter:
    MODE = 0x010
    ERROR = 0x014
    POSITION_KP = 0x020
    VELOCITY_KP = 0x028        # what upstream write_position_kd() writes
    TORQUE_LIMIT = 0x030
    TORQUE_TARGET = 0x044
    POSITION_MEASURED = 0x060
    BUS_VOLTAGE = 0x100


FUNC_POS = 7
DEVICE_MASK = 0x7F

# CAN error-frame class bits (linux/can/error.h)
CAN_ERR_CRTL = 0x04
CAN_ERR_PROT = 0x08
CAN_ERR_ACK = 0x20
CAN_ERR_BUSOFF = 0x40


def make_id(function: int, device_id: int) -> int:
    return (function << FUNC_POS) | (device_id & DEVICE_MASK)


def split_id(arbitration_id: int) -> tuple[int, int]:
    return arbitration_id >> FUNC_POS, arbitration_id & DEVICE_MASK


def link_info(channel: str) -> tuple[bool, bool, int | None]:
    """(present, up, ifindex) from sysfs, no privileges needed."""
    base = f"/sys/class/net/{channel}"
    try:
        with open(f"{base}/ifindex") as f:
            ifindex = int(f.read().strip())
        with open(f"{base}/flags") as f:
            flags = int(f.read().strip(), 16)
    except (OSError, ValueError):
        return False, False, None
    return True, bool(flags & 0x1), ifindex


def controller_state(channel: str) -> str:
    """ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF / STOPPED, or '' if unknown."""
    try:
        out = subprocess.run(["ip", "-details", "-json", "link", "show", channel],
                             capture_output=True, text=True, timeout=1.0)
        data = json.loads(out.stdout or "[]")
        return data[0]["linkinfo"]["info_data"].get("state", "")
    except (OSError, ValueError, KeyError, IndexError, subprocess.SubprocessError):
        return ""


class RecoilBus:
    """One CAN channel. `transport` is anything with python-can's
    send(msg) / recv(timeout) / shutdown() (a SocketcanBus, or FakeCanBus)."""

    # After this long with frames going out and nothing at all coming back the
    # bus is "quiet": nobody is ACKing (motor power off, cable out). Sending
    # into a quiet bus only grows the controller's error counters until it
    # goes ERROR-PASSIVE / BUS-OFF, and the gs_usb adapters cannot recover from
    # bus-off by themselves -- so a quiet bus gets one probe per second only.
    QUIET_AFTER = 1.0
    PROBE_PERIOD = 1.0
    # ENOBUFS = the kernel tx queue is full = frames are not leaving.
    BLOCKED_BACKOFF = 0.5

    def __init__(self, channel: str, transport) -> None:
        self.channel = channel
        self.transport = transport
        self.pdo: dict[int, tuple[float, float, float]] = {}   # dev -> (pos, vel, t)
        self.sdo: dict[int, tuple[bytes, float]] = {}            # dev -> (data, t)
        self.pong: dict[int, float] = {}
        self.extra_handlers = []   # callables(msg) for non-recoil frames (hand node)
        self.tx_dropped = 0
        self.error_frames = 0
        self.last_rx = time.monotonic()
        self.last_tx_ok = 0.0
        self.first_tx = None
        self.blocked_until = 0.0
        self.last_probe = 0.0

    # ------------------------------------------------------------- state

    @property
    def quiet(self) -> bool:
        now = time.monotonic()
        if self.first_tx is None or now - self.first_tx < self.QUIET_AFTER:
            return False
        return now - self.last_rx > self.QUIET_AFTER

    def may_send(self) -> bool:
        """False while the tx queue is jammed. A quiet bus still may send, but
        callers should restrict themselves to probe() then."""
        return time.monotonic() >= self.blocked_until

    def probe_due(self) -> bool:
        now = time.monotonic()
        if now - self.last_probe >= self.PROBE_PERIOD:
            self.last_probe = now
            return True
        return False

    def close(self) -> None:
        try:
            self.transport.shutdown()
        except Exception:  # noqa: BLE001 - closing a vanished adapter
            pass

    # ------------------------------------------------------------- tx

    def _send(self, function: int, device_id: int, data: bytes = b"") -> bool:
        if not self.may_send():
            return False
        msg = can.Message(arbitration_id=make_id(function, device_id),
                          is_extended_id=False, data=data)
        try:
            self.transport.send(msg, timeout=0)
        except can.CanError as error:
            self.tx_dropped += 1
            err = getattr(error, "error_code", None)
            if err in (errno.ENOBUFS, errno.EAGAIN) or "105" in str(error) \
                    or "No buffer space" in str(error):
                self.blocked_until = time.monotonic() + self.BLOCKED_BACKOFF
                return False
            if err == errno.ENETDOWN or "Network is down" in str(error):
                raise OSError(errno.ENETDOWN, f"{self.channel} is down") from error
            if err == errno.ENODEV or "No such device" in str(error):
                raise OSError(errno.ENODEV, f"{self.channel} vanished") from error
            self.blocked_until = time.monotonic() + self.BLOCKED_BACKOFF
            return False
        now = time.monotonic()
        if self.first_tx is None:
            self.first_tx = now
            self.last_rx = max(self.last_rx, now)
        self.last_tx_ok = now
        return True

    def send_pdo2(self, device_id: int, position: float, velocity: float) -> bool:
        return self._send(Function.RECEIVE_PDO_2, device_id,
                          struct.pack("<ff", position, velocity))

    def set_mode(self, device_id: int, mode: int) -> bool:
        return self._send(Function.NMT, device_id, struct.pack("<BB", mode, device_id))

    def feed(self, device_id: int) -> bool:
        return self._send(Function.HEARTBEAT, device_id)

    def ping(self, device_id: int) -> bool:
        return self._send(Function.RECEIVE_PDO_1, device_id, b"\xCA")

    def request_f32(self, device_id: int, param: int) -> bool:
        return self._send(Function.RECEIVE_SDO, device_id, struct.pack("<BH", 0x02 << 5, param))

    def write_f32(self, device_id: int, param: int, value: float) -> bool:
        return self._send(Function.RECEIVE_SDO, device_id,
                          struct.pack("<BHB", 0x01 << 5, param, 0) + struct.pack("<f", value))

    def send_raw(self, arbitration_id: int, data: bytes) -> bool:
        """For the hand node, which shares the socket."""
        if not self.may_send():
            return False
        msg = can.Message(arbitration_id=arbitration_id, is_extended_id=False, data=data)
        try:
            self.transport.send(msg, timeout=0)
        except can.CanError:
            self.tx_dropped += 1
            self.blocked_until = time.monotonic() + self.BLOCKED_BACKOFF
            return False
        return True

    # ------------------------------------------------------------- rx

    def _dispatch(self, msg) -> None:
        if msg.is_error_frame:
            self.error_frames += 1
            return
        self.last_rx = time.monotonic()
        function, device_id = split_id(msg.arbitration_id)
        data = bytes(msg.data)
        if function == Function.TRANSMIT_PDO_2 and len(data) >= 8:
            pos, vel = struct.unpack("<ff", data[:8])
            self.pdo[device_id] = (pos, vel, self.last_rx)
        elif function == Function.TRANSMIT_SDO and len(data) >= 4:
            self.sdo[device_id] = (data[:4], self.last_rx)
        elif function == Function.TRANSMIT_PDO_1 and data[:1] == b"\xCA":
            self.pong[device_id] = self.last_rx
        else:
            for handler in self.extra_handlers:
                handler(msg)

    def collect(self, deadline: float, waiting: set[int] | None = None) -> None:
        """Read frames until `deadline` (monotonic) or until every device in
        `waiting` has delivered a fresh PDO_2 reply (the set is consumed)."""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            try:
                msg = self.transport.recv(timeout=remaining)
            except can.CanError as error:
                if "Network is down" in str(error) or "No such device" in str(error):
                    raise OSError(errno.ENETDOWN, f"{self.channel}: {error}") from error
                return
            if msg is None:
                return
            self._dispatch(msg)
            if waiting is not None and not msg.is_error_frame:
                function, device_id = split_id(msg.arbitration_id)
                if function == Function.TRANSMIT_PDO_2:
                    waiting.discard(device_id)
                    if not waiting:
                        return

    def drain(self) -> None:
        self.collect(time.monotonic())   # deadline already passed -> one poll
        while True:
            try:
                msg = self.transport.recv(timeout=0)
            except can.CanError:
                return
            if msg is None:
                return
            self._dispatch(msg)


def open_socketcan(channel: str):
    """A python-can socketcan transport. Error frames are delivered (python-can
    enables CAN_RAW_ERR_FILTER), own frames are not."""
    return can.interface.Bus(interface="socketcan", channel=channel,
                             receive_own_messages=False)


def sdo_f32(data: bytes) -> float:
    return struct.unpack("<f", data[:4])[0]


__all__ = ["Function", "Mode", "Parameter", "RecoilBus", "link_info", "controller_state",
           "open_socketcan", "sdo_f32", "make_id", "split_id"]
