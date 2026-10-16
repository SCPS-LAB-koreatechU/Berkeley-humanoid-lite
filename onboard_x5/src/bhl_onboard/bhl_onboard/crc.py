"""CRC for LowCmd, the role unitree's Crc32Core plays for rt/lowcmd.

zlib CRC-32 over a fixed little-endian packing of mode_machine and every motor
command. Any client (Python or C++) can reproduce it from this layout:
  <B  mode_machine
  22 x <B5fI  mode, q, dq, tau, kp, kd, reserve
"""

from __future__ import annotations

import struct
import zlib

_HEAD = struct.Struct("<B")
_MOTOR = struct.Struct("<B5fI")


def lowcmd_crc(msg) -> int:
    parts = [_HEAD.pack(msg.mode_machine & 0xFF)]
    for c in msg.motor_cmd:
        parts.append(_MOTOR.pack(c.mode & 0xFF, c.q, c.dq, c.tau, c.kp, c.kd, c.reserve & 0xFFFFFFFF))
    return zlib.crc32(b"".join(parts)) & 0xFFFFFFFF


def seal(msg):
    """Fill msg.crc in place and return msg."""
    msg.crc = lowcmd_crc(msg)
    return msg
