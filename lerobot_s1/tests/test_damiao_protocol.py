# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Damiao driver: frame encoding and bus behaviour against the fake CAN bus."""

import math

import pytest

from lerobot_robot_s1.motors import damiao as dm
from lerobot_robot_s1.motors.damiao import DamiaoBus, DamiaoMotor, decode_status, encode_mit

from .conftest import FakeCanBus, make_fake_motors

LIMITS = dm.MODEL_LIMITS["J4310"]


def test_encode_mit_zero_command_is_midscale():
    data = encode_mit(0.0, 0.0, 0.0, 0.0, 0.0, LIMITS)
    assert len(data) == 8
    # p_des = 0 rad -> 0x7FFF; v_des = 0 -> 0x7FF; kp = kd = 0; t_ff = 0 -> 0x7FF
    assert data[0] == 0x7F and data[1] == 0xFF
    assert data[2] == 0x7F and (data[3] >> 4) == 0xF
    assert (data[3] & 0x0F) == 0 and data[4] == 0 and data[5] == 0 and (data[6] >> 4) == 0
    assert (data[6] & 0x0F) == 0x7 and data[7] == 0xFF


def test_encode_mit_saturates_out_of_range_inputs():
    data = encode_mit(99.0, -99.0, 9999.0, 9999.0, 99.0, LIMITS)
    assert data[0] == 0xFF and data[1] == 0xFF  # p clipped to +p_max
    assert data[2] == 0x00 and (data[3] >> 4) == 0  # v clipped to -v_max
    assert (data[3] & 0x0F) == 0xF and data[4] == 0xFF  # kp = 500
    assert data[7] == 0xFF and (data[6] & 0x0F) == 0xF  # t_ff = +t_max


@pytest.mark.parametrize("pos", [-12.5, -1.234, 0.0, 0.5, 12.5])
def test_status_roundtrip_position(pos):
    p_max, v_max, t_max = LIMITS
    p = dm.float_to_uint(pos, -p_max, p_max, 16)
    v = dm.float_to_uint(2.0, -v_max, v_max, 12)
    t = dm.float_to_uint(-1.0, -t_max, t_max, 12)
    data = bytes(
        [0x11, (p >> 8) & 0xFF, p & 0xFF, (v >> 4) & 0xFF, ((v & 0xF) << 4) | (t >> 8), t & 0xFF, 42, 0]
    )
    status, q, dq, tau, temp = decode_status(data, LIMITS)
    assert status == 1
    assert math.isclose(q, pos, abs_tol=dm.MODEL_LIMITS["J4310"][0] * 2 / 65535)
    assert math.isclose(dq, 2.0, abs_tol=v_max * 2 / 4095)
    assert math.isclose(tau, -1.0, abs_tol=t_max * 2 / 4095)
    assert temp == 42


def _bus(patch_can, positions=None, **kwargs) -> tuple[DamiaoBus, FakeCanBus]:
    models = {"a": "J8009P", "b": "J4310"}
    ids = {"a": 1, "b": 2}
    fake = patch_can(FakeCanBus(make_fake_motors(models, ids, positions), **kwargs))
    bus = DamiaoBus(
        {n: DamiaoMotor(can_id=ids[n], model=models[n], kp=20.0, kd=1.0, torque_limit=5.0) for n in models},
        interface="virtual",
        channel="test",
    )
    return bus, fake


def test_connect_pings_and_reads_limits(patch_can):
    bus, fake = _bus(patch_can, positions={"a": 0.3, "b": -0.2})
    bus.connect()
    try:
        assert bus.is_connected
        assert bus.limits["a"] == dm.MODEL_LIMITS["J8009P"]
        pos = bus.read_positions()
        assert pos["a"] == pytest.approx(0.3, abs=1e-3)
        assert pos["b"] == pytest.approx(-0.2, abs=1e-3)
        # torque-free motors were probed with zero-gain frames, not enabled
        assert dm.CMD_ENABLE not in fake.simple_commands(1)
    finally:
        bus.disconnect()
    assert fake.shut_down
    assert not bus.is_connected


def test_connect_fails_when_motor_silent(patch_can):
    bus, fake = _bus(patch_can)
    del fake._by_id[2]  # motor "b" never answers
    with pytest.raises(ConnectionError, match="b \\(id 2\\)"):
        bus.connect()
    assert not bus.is_connected


def test_read_falls_back_to_refresh_when_disabled_motor_ignores_mit(patch_can):
    bus, fake = _bus(patch_can, positions={"a": 1.0}, reply_to_mit_when_disabled=False)
    bus.connect()
    try:
        assert bus.read_positions()["a"] == pytest.approx(1.0, abs=1e-3)
        refreshes = [d for d in fake.frames_to(dm.BROADCAST_ID) if d[2] == dm.CMD_REFRESH]
        assert len(refreshes) >= 2
    finally:
        bus.disconnect()


def test_enable_holds_present_position_and_write_moves(patch_can):
    bus, fake = _bus(patch_can, positions={"a": 0.5, "b": -0.5})
    bus.connect()
    try:
        bus.enable_torque()
        assert fake.simple_commands(1)[:1] == [dm.CMD_ENABLE]
        assert bus.is_holding("a") and bus.is_holding("b")
        assert bus.last_goal["a"] == pytest.approx(0.5, abs=1e-3)
        # limp frames (kp=0) came before the hold frame (kp>0)
        mit = [d for d in fake.frames_to(1) if d[:7] != b"\xff" * 7]
        kps = [((d[3] & 0x0F) << 8) | d[4] for d in mit]
        assert kps[0] == 0 and kps[-1] > 0
        sent = bus.write_positions({"a": 0.8})
        assert sent == {"a": pytest.approx(0.8)}
        assert bus.read_positions()["a"] == pytest.approx(0.8, abs=1e-3)
    finally:
        bus.disconnect()
    # disconnect disables torque (sent twice on purpose)
    assert fake.simple_commands(1).count(dm.CMD_DISABLE) >= 2
    assert not fake.motors["a"].enabled


def test_write_requires_torque_enabled(patch_can):
    bus, _ = _bus(patch_can)
    bus.connect()
    try:
        with pytest.raises(RuntimeError, match="not torque-enabled"):
            bus.write_positions({"a": 0.1})
    finally:
        bus.disconnect()


def test_torque_limit_makes_joint_yield(patch_can):
    bus, fake = _bus(patch_can, positions={"a": 0.0, "b": 0.0})
    bus.connect()
    try:
        bus.enable_torque()
        fake.motors["a"].torque = 7.0  # above the 5 Nm limit
        bus.read_positions()  # refresh the cached torque
        sent = bus.write_positions({"a": 1.0, "b": 1.0})
        assert sent["a"] == pytest.approx(0.0, abs=1e-3)  # yielded to present position
        assert sent["b"] == pytest.approx(1.0)
        # a frame flagged with an error nibble is not trusted for yielding
        fake.motors["a"].error = 0x9
        bus.read_positions()
        assert bus.states["a"].error
        sent = bus.write_positions({"a": 1.0})
        assert sent["a"] == pytest.approx(1.0)
    finally:
        bus.disconnect()


def test_set_zero_position_requires_torque_off(patch_can):
    bus, fake = _bus(patch_can, positions={"a": 0.7})
    bus.connect()
    try:
        bus.enable_torque("a")
        with pytest.raises(RuntimeError, match="Disable torque"):
            bus.set_zero_position("a")
        bus.disable_torque("a")
        bus.set_zero_position("a")
        assert fake.motors["a"].zero_calls == 1
        assert bus.read_positions(["a"])["a"] == pytest.approx(0.0, abs=1e-3)
    finally:
        bus.disconnect()
