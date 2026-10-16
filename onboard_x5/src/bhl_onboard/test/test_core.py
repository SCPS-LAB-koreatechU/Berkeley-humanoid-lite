"""Core behaviour against simulated recoil motors (no ROS needed)."""

import math
import os
import struct
import time

import pytest

from bhl_onboard import config as config_mod
from bhl_onboard import core as core_mod
from bhl_onboard import recoil
from bhl_onboard.crc import lowcmd_crc, seal
from bhl_onboard.fake_can import sim_factory

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "bhl_x5.yaml")


def make(**overrides):
    cfg = config_mod.load(CONFIG, offsets_file=None)
    for section, values in overrides.items():
        cfg.raw.setdefault(section, {}).update(values)
    factory = sim_factory(cfg)
    for channel in cfg.channels:
        factory(channel)      # create the simulated motors up front
    log = []
    core = core_mod.Core(cfg, factory, sim=True, log=log.append)
    core.log_lines = log
    return core, factory


def run(core, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        core.step()
        time.sleep(core.period)


def motor(core, name):
    return next(m for m in core.motors if m.spec.name == name)


def fake(factory, core, name):
    m = motor(core, name)
    return factory.buses[m.spec.channel].motors[m.spec.device_id]


def command(core, targets, **fields):
    cmd = core_mod.empty_lowcmd(time.monotonic())
    for name, q in targets.items():
        slot = motor(core, name).spec.slot
        cmd.mode[slot] = 1
        cmd.q[slot] = q
        for key, value in fields.items():
            getattr(cmd, key)[slot] = value
    core.set_lowcmd(cmd)


def stream(core, targets, seconds, **fields):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        command(core, targets, **fields)
        core.step()
        time.sleep(core.period)


def enable(core):
    ok, text = core.call_now("enable")
    assert ok, text
    return text


@pytest.fixture(autouse=True)
def _call_now(monkeypatch):
    # In tests there is no control thread: run the request inside one step.
    def call_now(self, name):
        import threading
        box, done = [], threading.Event()
        self.requests.put((name, box, done))
        self.step()
        return box[0]
    monkeypatch.setattr(core_mod.Core, "call_now", call_now, raising=False)


YAW = "arm_left_shoulder_yaw_joint"
PITCH = "arm_left_shoulder_pitch_joint"


def test_idle_reads_every_present_motor_without_driving():
    core, factory = make()
    run(core, 0.3)
    for m in core.motors:
        if m.spec.present:
            assert m.seen, m.spec.name
            assert not m.live
    for bus in factory.buses.values():
        for fm in bus.motors.values():
            assert recoil.Mode.POSITION not in fm.mode_history


def test_enable_holds_measured_pose_and_skips_disabled_motor():
    core, factory = make()
    fake(factory, core, YAW).position = motor(core, YAW).spec.to_actuator(0.3)
    run(core, 0.3)
    text = enable(core)
    assert core.state == core_mod.ACTIVE
    assert "9 motors" in text
    yaw = motor(core, YAW)
    assert yaw.live and yaw.cmd == pytest.approx(0.3, abs=1e-4)
    # CAN 8 is enabled: false -> never energised
    assert recoil.Mode.POSITION not in fake(factory, core, "arm_right_elbow_pitch_joint").mode_history
    run(core, 0.2)
    assert yaw.q == pytest.approx(0.3, abs=1e-3)       # no jump at enable


def test_follows_lowcmd_with_slew_limit_and_clamp():
    core, factory = make()
    run(core, 0.2)
    enable(core)
    yaw = motor(core, YAW)
    stream(core, {YAW: 0.5}, 0.2)
    # 1 rad/s slew: after 0.2 s the target cannot be past ~0.2 rad
    assert 0.1 < yaw.cmd < 0.25
    stream(core, {YAW: 0.5}, 0.6)
    assert yaw.cmd == pytest.approx(0.5, abs=1e-6)
    assert yaw.q == pytest.approx(0.5, abs=0.02)
    # beyond the URDF limit (0.785) less margin 0.05 -> clamped
    stream(core, {YAW: 3.0}, 0.6)
    assert yaw.cmd == pytest.approx(0.785398 - 0.05, abs=1e-4)
    assert yaw.fault & core_mod.FAULT_CLAMPED
    # what reached the motor is the converted actuator value
    assert fake(factory, core, YAW).target == pytest.approx(yaw.spec.to_actuator(yaw.cmd), abs=1e-4)


def test_command_loss_holds_then_damps():
    core, factory = make(command={"timeout": 0.1, "hold_then_damp": 0.2})
    run(core, 0.2)
    enable(core)
    stream(core, {YAW: 0.2}, 0.6)
    held = motor(core, YAW).cmd
    assert held == pytest.approx(0.2, abs=1e-6)
    run(core, 0.15)                     # silent, inside hold window
    assert core.state == core_mod.ACTIVE
    assert motor(core, YAW).cmd == pytest.approx(held, abs=1e-6)
    run(core, 0.3)
    assert core.state == core_mod.DAMPING
    assert fake(factory, core, YAW).mode == recoil.Mode.DAMPING


def test_command_from_before_enable_is_ignored():
    core, factory = make()
    run(core, 0.2)
    command(core, {YAW: 0.6})           # stale: published before enable
    enable(core)
    run(core, 0.3)
    assert motor(core, YAW).cmd == pytest.approx(0.0, abs=1e-3)


def test_estop_latches():
    core, factory = make()
    run(core, 0.2)
    enable(core)
    ok, _ = core.call_now("estop")
    assert ok and core.state == core_mod.ESTOP
    assert fake(factory, core, YAW).mode == recoil.Mode.DAMPING
    ok, text = core.call_now("enable")
    assert not ok and "E-STOP" in text
    ok, _ = core.call_now("reset_estop")
    assert ok and core.state == core_mod.IDLE
    assert fake(factory, core, YAW).mode == recoil.Mode.IDLE
    enable(core)


def test_quiet_bus_sends_nothing_at_enable():
    core, factory = make()
    run(core, 0.1)
    bus = factory.buses["can0"]
    bus.powered = False
    run(core, 1.3)                      # > RecoilBus.QUIET_AFTER
    assert core.channels["can0"].bus.quiet
    bus.keep_sent = True
    bus.sent.clear()
    ok, text = core.call_now("enable")
    assert ok                            # right arm still came up
    assert "quiet" in text
    assert not any(recoil.split_id(m.arbitration_id)[0] == recoil.Function.NMT for m in bus.sent)
    # while quiet: at most one probe per second
    bus.sent.clear()
    run(core, 1.0)
    assert len(bus.sent) <= 2
    assert any("nothing answers" in f for f in core.faults())


def test_missing_motor_is_skipped_others_come_up():
    core, factory = make()
    fake(factory, core, PITCH).present = False
    run(core, 0.3)
    ok, text = core.call_now("enable")
    assert ok and PITCH in text and "no answer" in text
    assert motor(core, PITCH).fault & core_mod.FAULT_LOST
    assert motor(core, YAW).live


def test_out_of_range_motor_refused():
    core, factory = make()
    m = motor(core, YAW)
    fake(factory, core, YAW).position = m.spec.to_actuator(1.2)   # limit 0.785
    run(core, 0.3)
    ok, text = core.call_now("enable")
    assert ok and "outside" in text
    assert not m.live and m.fault & core_mod.FAULT_OUT_OF_RANGE


def test_motor_lost_while_active_drops_to_idle():
    core, factory = make(safety={"pdo_loss_threshold": 20})
    run(core, 0.2)
    enable(core)
    fake(factory, core, YAW).present = False
    stream(core, {}, 0.3)
    yaw = motor(core, YAW)
    assert not yaw.live and yaw.fault & core_mod.FAULT_LOST
    assert motor(core, PITCH).live                  # the rest keeps going


def test_gains_from_lowcmd_are_capped_and_rate_limited():
    core, factory = make()
    run(core, 0.2)
    enable(core)
    fm = fake(factory, core, YAW)
    fm.writes.clear()
    stream(core, {YAW: 0.0}, 0.5, kp=9.0, kd=0.02)
    kp_writes = [v for p, v in fm.writes if p == recoil.Parameter.POSITION_KP]
    assert kp_writes and max(kp_writes) == pytest.approx(1.5)   # caps.position_kp
    assert fm.params[recoil.Parameter.VELOCITY_KP] == pytest.approx(0.02)
    n = len(fm.writes)
    stream(core, {YAW: 0.0}, 0.5, kp=9.0, kd=0.02)
    assert len(fm.writes) == n          # unchanged gains are not rewritten


def test_bus_loss_and_reopen():
    core, factory = make()
    run(core, 0.2)
    enable(core)
    factory.buses["can0"].closed = True
    stream(core, {}, 0.1)
    yaw = motor(core, YAW)
    assert not yaw.live and yaw.fault & core_mod.FAULT_BUS
    run(core, 0.8)                       # link check reopens it
    assert core.channels["can0"].bus is not None
    assert not yaw.fault & core_mod.FAULT_BUS
    assert yaw.seen


def test_zero_writes_offsets(tmp_path, monkeypatch):
    target = tmp_path / "offsets.yaml"
    monkeypatch.setattr(core_mod, "save_offsets",
                        lambda offsets: config_mod.save_offsets(offsets, target))
    core, factory = make()
    fake(factory, core, YAW).position += 0.75      # rotor rad
    run(core, 0.3)
    ok, text = core.call_now("zero")
    assert ok, text
    assert motor(core, YAW).spec.from_actuator(motor(core, YAW).raw) == pytest.approx(0.0, abs=1e-6)
    cfg = config_mod.load(CONFIG, offsets_file=target)
    yaw = next(m for m in cfg.motors if m.name == YAW)
    assert yaw.from_actuator(fake(factory, core, YAW).position) == pytest.approx(0.0, abs=1e-5)


def test_shutdown_leaves_motors_idle():
    core, factory = make()
    run(core, 0.2)
    enable(core)
    core.shutdown()
    assert fake(factory, core, YAW).mode == recoil.Mode.IDLE


def test_crc():
    from bhl_msgs.msg import LowCmd
    msg = LowCmd()
    msg.motor_cmd[3].q = 0.25
    seal(msg)
    assert msg.crc == lowcmd_crc(msg)
    msg.motor_cmd[3].q = 0.26
    assert msg.crc != lowcmd_crc(msg)


def test_frame_layout_matches_upstream_recoil():
    # upstream: can_id = func_id << 7 | device_id ; PDO_2 = <ff
    assert recoil.make_id(recoil.Function.RECEIVE_PDO_2, 5) == (0b0110 << 7) | 5
    assert recoil.split_id((0b0101 << 7) | 9) == (recoil.Function.TRANSMIT_PDO_2, 9)
    assert struct.pack("<BHB", 0x01 << 5, recoil.Parameter.POSITION_KP, 0) == b"\x20\x20\x00\x00"


def test_config_rejects_duplicate_ids(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("buses: {left: can0}\nmotors:\n"
                   "  - {slot: 0, name: a, bus: left, device_id: 1}\n"
                   "  - {slot: 1, name: b, bus: left, device_id: 1}\n")
    with pytest.raises(ValueError, match="share"):
        config_mod.load(bad, offsets_file=None)


def test_plan_sampling_hits_points():
    from bhl_onboard.commander_node import Plan
    plan = Plan(refs=[("m", 0)], times=[0.0, 1.0, 2.0], positions=[[0.0], [1.0], [0.0]],
                velocities=[[0.0], [0.0], [0.0]], start=0.0, final_tolerance=0.01)
    assert plan.sample(1.0)[0][0] == pytest.approx(1.0)
    assert plan.sample(0.5)[0][0] == pytest.approx(0.5)
    pos, vel, done = plan.sample(3.0)
    assert pos == [0.0] and done
    assert math.isfinite(plan.sample(1.5)[1][0])
