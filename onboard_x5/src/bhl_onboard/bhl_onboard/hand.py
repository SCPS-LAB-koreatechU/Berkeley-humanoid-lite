"""Hand + wrist units (one ESP32 per side, 17 motors).

The board treats a hand as 17 position slots. Drivers:
  NullHand   -- sends nothing, reports offline (default: safe on a shared bus)
  SimHand    -- first-order echo of the targets, for sim and tests
  Esp32Hand  -- the real node. Its CAN protocol is not defined yet; see the
                TODO block. It must not reuse the recoil id space on the same
                bus: recoil owns every 11-bit id whose low 7 bits are a motor
                id in use (function << 7 | device_id).
"""

from __future__ import annotations

import math
import time

from .config import NUM_HAND_MOTOR, HandSpec


class HandDriver:
    online = False

    def __init__(self, spec: HandSpec) -> None:
        self.spec = spec
        self.q = [0.0] * NUM_HAND_MOTOR
        self.dq = [0.0] * NUM_HAND_MOTOR
        self.target = [0.0] * NUM_HAND_MOTOR
        self.energised = False

    def set_energised(self, on: bool) -> None:
        self.energised = on

    def exchange(self) -> None:
        """Called from the control loop at control.rate_hz / hand_divider."""


class NullHand(HandDriver):
    pass


class SimHand(HandDriver):
    TIME_CONSTANT = 0.08

    def __init__(self, spec: HandSpec) -> None:
        super().__init__(spec)
        self.online = True
        self.stamp = time.monotonic()

    def exchange(self) -> None:
        now = time.monotonic()
        dt = min(now - self.stamp, 0.1)
        self.stamp = now
        if not self.energised:
            self.dq = [0.0] * NUM_HAND_MOTOR
            return
        alpha = 1.0 - math.exp(-dt / self.TIME_CONSTANT)
        for i in range(NUM_HAND_MOTOR):
            new = self.q[i] + (self.target[i] - self.q[i]) * alpha
            self.dq[i] = (new - self.q[i]) / dt if dt > 0 else 0.0
            self.q[i] = new


class Esp32Hand(HandDriver):
    """TODO(protocol): fill in once the ESP32 firmware's frames are fixed.

    What the board needs from the protocol:
      * target frame(s): 17 positions -- e.g. int16 centi-radians, 4 per frame
        -> 5 frames per cycle (at 50 Hz that is 250 frames/s, ~3% of 1 Mbit/s)
      * state frame(s): 17 positions (+ fault bits) back
      * torque on/off command and a heartbeat/timeout on the ESP32 side, so a
        dead board makes the hand go limp rather than hold forever
    Wire it with bus.send_raw(...) in exchange() and a handler appended to
    bus.extra_handlers for the replies (frames recoil does not claim).
    """

    def __init__(self, spec: HandSpec, bus) -> None:
        super().__init__(spec)
        raise NotImplementedError(
            f"hands.{spec.side}.driver=esp32: the ESP32 CAN protocol is not implemented yet "
            "(bhl_onboard/hand.py Esp32Hand). Use driver: none or sim.")


def make_hand(spec: HandSpec, bus=None, sim: bool = False) -> HandDriver:
    if spec.driver == "sim" or (sim and spec.driver == "esp32"):
        return SimHand(spec)
    if spec.driver == "esp32":
        return Esp32Hand(spec, bus)
    if spec.driver == "none":
        return NullHand(spec)
    raise ValueError(f"hands.{spec.side}.driver: unknown '{spec.driver}'")
