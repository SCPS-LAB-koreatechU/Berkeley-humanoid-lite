"""Board configuration (config/bhl_x5.yaml) as plain objects."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path

import yaml

NUM_MOTOR = 22
NUM_HAND_MOTOR = 17
OFFSETS_FILE = Path(os.environ.get("BHL_OFFSETS_FILE",
                                   Path.home() / ".config" / "bhl_onboard" / "offsets.yaml"))


@dataclass
class MotorSpec:
    slot: int
    name: str
    bus: str | None = None
    channel: str | None = None
    device_id: int | None = None
    direction: float = 1.0
    gear_ratio: float = 1.0
    offset: float = 0.0
    lower: float = -math.pi
    upper: float = math.pi
    enabled: bool = False
    gains: dict = field(default_factory=dict)

    @property
    def present(self) -> bool:
        return self.device_id is not None and self.channel is not None

    def to_actuator(self, q: float) -> float:
        return (q + self.offset) * self.direction * self.gear_ratio

    def to_actuator_velocity(self, dq: float) -> float:
        return dq * self.direction * self.gear_ratio

    def from_actuator(self, p: float) -> float:
        return p / self.gear_ratio * self.direction - self.offset

    def from_actuator_velocity(self, v: float) -> float:
        return v / self.gear_ratio * self.direction


@dataclass
class HandSpec:
    side: str
    bus: str | None
    channel: str | None
    driver: str
    joints: list[str]
    lower: list[float]
    upper: list[float]
    esp32: dict


@dataclass
class Config:
    raw: dict
    motors: list[MotorSpec]
    hands: dict[str, HandSpec]
    path: Path

    def section(self, name: str) -> dict:
        return dict(self.raw.get(name) or {})

    @property
    def channels(self) -> list[str]:
        out = []
        for m in self.motors:
            if m.present and m.channel not in out:
                out.append(m.channel)
        for h in self.hands.values():
            if h.driver == "esp32" and h.channel and h.channel not in out:
                out.append(h.channel)
        return out


def _per_slot(value, n: int, default: float) -> list[float]:
    if value is None:
        return [default] * n
    if isinstance(value, (int, float)):
        return [float(value)] * n
    values = [float(v) for v in value]
    if len(values) != n:
        raise ValueError(f"expected {n} values, got {len(values)}")
    return values


def load(path: str | Path, offsets_file: Path | None = OFFSETS_FILE) -> Config:
    path = Path(path)
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    buses = dict(raw.get("buses") or {})

    motors = [MotorSpec(slot=i, name=f"slot_{i}") for i in range(NUM_MOTOR)]
    seen = set()
    for entry in raw.get("motors") or []:
        slot = int(entry["slot"])
        if not 0 <= slot < NUM_MOTOR:
            raise ValueError(f"motor slot {slot} out of range 0..{NUM_MOTOR - 1}")
        if slot in seen:
            raise ValueError(f"motor slot {slot} listed twice")
        seen.add(slot)
        bus = entry.get("bus")
        if entry.get("device_id") is not None and bus not in buses:
            raise ValueError(f"{entry['name']}: bus '{bus}' not in `buses`")
        gear = float(entry.get("gear_ratio", 1.0))
        if gear == 0.0:
            raise ValueError(f"{entry['name']}: gear_ratio must not be 0")
        motors[slot] = MotorSpec(
            slot=slot, name=str(entry["name"]), bus=bus,
            channel=buses.get(bus) if bus else None,
            device_id=None if entry.get("device_id") is None else int(entry["device_id"]),
            direction=float(entry.get("direction", 1.0)), gear_ratio=gear,
            offset=float(entry.get("offset", 0.0)),
            lower=float(entry.get("lower", -math.pi)), upper=float(entry.get("upper", math.pi)),
            enabled=bool(entry.get("enabled", False)), gains=dict(entry.get("gains") or {}))

    ids = {}
    for m in motors:
        if m.present:
            key = (m.channel, m.device_id)
            if key in ids:
                raise ValueError(f"{m.name} and {ids[key]} share {m.channel} id {m.device_id}")
            ids[key] = m.name

    if offsets_file is not None and Path(offsets_file).exists():
        with open(offsets_file) as f:
            saved = yaml.safe_load(f) or {}
        for m in motors:
            if m.name in saved:
                m.offset = float(saved[m.name])

    hands = {}
    for side, entry in (raw.get("hands") or {}).items():
        joints = list(entry.get("joints") or [f"{side}_hand_m{i:02d}" for i in range(NUM_HAND_MOTOR)])
        if len(joints) != NUM_HAND_MOTOR:
            raise ValueError(f"hands.{side}.joints: need {NUM_HAND_MOTOR} names")
        bus = entry.get("bus")
        hands[side] = HandSpec(
            side=side, bus=bus, channel=buses.get(bus) if bus else None,
            driver=str(entry.get("driver", "none")), joints=joints,
            lower=_per_slot(entry.get("lower"), NUM_HAND_MOTOR, -math.pi),
            upper=_per_slot(entry.get("upper"), NUM_HAND_MOTOR, math.pi),
            esp32=dict(entry.get("esp32") or {}))
    return Config(raw=raw, motors=motors, hands=hands, path=path)


def save_offsets(offsets: dict[str, float], offsets_file: Path = OFFSETS_FILE) -> Path:
    offsets_file = Path(offsets_file)
    offsets_file.parent.mkdir(parents=True, exist_ok=True)
    current = {}
    if offsets_file.exists():
        with open(offsets_file) as f:
            current = yaml.safe_load(f) or {}
    current.update({k: round(float(v), 6) for k, v in offsets.items()})
    with open(offsets_file, "w") as f:
        yaml.safe_dump(current, f, sort_keys=True)
    return offsets_file
