# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""S1Follower with the motor bus and camera mocked."""

import numpy as np
import pytest
from lerobot.utils.errors import DeviceNotConnectedError

from lerobot_robot_s1 import S1Follower, S1FollowerConfig
from lerobot_robot_s1 import s1_follower as follower_mod
from lerobot_robot_s1.calibration import make_calibration
from lerobot_robot_s1.constants import FOLLOWER_CAN_IDS, FOLLOWER_MOTOR_MODELS, JOINT_LIMITS, JOINTS
from lerobot_robot_s1.motors import damiao as dm

from .conftest import FakeCamera, FakeCanBus, make_fake_motors

ZERO = 0.0
CLOSED_GRIPPER = -5.0  # motor rad at fully closed


def write_calibration(robot: S1Follower) -> None:
    robot.calibration = {
        j: make_calibration(
            j,
            FOLLOWER_CAN_IDS[j],
            inverted=False,
            zero_rad=ZERO,
            motor_min_rad=CLOSED_GRIPPER if j == "gripper" else JOINT_LIMITS[j][0],
            motor_max_rad=0.0 if j == "gripper" else JOINT_LIMITS[j][1],
        )
        for j in JOINTS
    }
    robot._save_calibration()


@pytest.fixture
def fake_cameras(monkeypatch):
    def factory(configs):
        return {name: FakeCamera(cfg.height, cfg.width) for name, cfg in configs.items()}

    monkeypatch.setattr(follower_mod, "make_cameras_from_configs", factory)


@pytest.fixture
def robot(tmp_path, patch_can, fake_cameras):
    fake = patch_can(FakeCanBus(make_fake_motors(FOLLOWER_MOTOR_MODELS, FOLLOWER_CAN_IDS)))
    cfg = S1FollowerConfig(id="test_arm", calibration_dir=tmp_path, can_interface="virtual", can_channel="x")
    r = S1Follower(cfg)
    write_calibration(r)
    r = S1Follower(cfg)  # reload from file, like a fresh process
    yield r, fake
    if r.is_connected:
        r.disconnect()


def test_features(robot):
    r, _ = robot
    assert list(r.action_features) == [f"{j}.pos" for j in JOINTS]
    assert r.observation_features == {**r.action_features, "wrist": (480, 640, 3)}


def test_uncalibrated_robot(tmp_path, patch_can, fake_cameras):
    patch_can(FakeCanBus(make_fake_motors(FOLLOWER_MOTOR_MODELS, FOLLOWER_CAN_IDS)))
    r = S1Follower(S1FollowerConfig(id="fresh", calibration_dir=tmp_path, can_interface="virtual"))
    assert not r.is_calibrated
    r.connect(calibrate=False)  # must not fail: this is what lerobot-calibrate does first
    assert r.is_connected
    with pytest.raises(RuntimeError, match="not calibrated"):
        r.configure()
    r.disconnect()


def test_connect_configure_observe_disconnect(robot):
    r, fake = robot
    assert r.is_calibrated
    with pytest.raises(DeviceNotConnectedError):
        r.get_observation()
    r.connect()
    assert r.is_connected
    for j in JOINTS:
        assert fake.motors[j].enabled
    obs = r.get_observation()
    assert set(obs) == set(r.observation_features)
    for j in JOINTS:
        assert obs[f"{j}.pos"] == pytest.approx(0.0, abs=1e-3)
    assert isinstance(obs["wrist"], np.ndarray) and obs["wrist"].shape == (480, 640, 3)
    r.disconnect()
    assert not r.is_connected
    for j in JOINTS:
        assert not fake.motors[j].enabled
        assert dm.CMD_DISABLE in fake.simple_commands(FOLLOWER_CAN_IDS[j])


def test_send_action_rate_limit_and_range_clamp(robot):
    r, fake = robot
    r.connect()
    # one step towards a far target is limited to max_relative_target
    sent = r.send_action({"base.pos": 1.0, "gripper.pos": -0.045})
    assert sent["base.pos"] == pytest.approx(r.config.max_relative_target, abs=1e-3)
    assert sent["gripper.pos"] == pytest.approx(-r.config.gripper_max_relative_target, abs=1e-4)
    assert fake.motors["base"].position == pytest.approx(r.config.max_relative_target, abs=1e-3)
    # keep stepping: the target converges on the goal, never past the URDF/calibrated limit
    for _ in range(200):
        sent = r.send_action({"base.pos": 5.0})
    assert sent["base.pos"] == pytest.approx(JOINT_LIMITS["base"][1])
    assert fake.motors["base"].position == pytest.approx(JOINT_LIMITS["base"][1], abs=1e-3)
    # gripper: metres in, closed motor position out
    for _ in range(20):
        sent = r.send_action({"gripper.pos": -1.0})
    assert sent["gripper.pos"] == pytest.approx(-0.045)
    assert fake.motors["gripper"].position == pytest.approx(CLOSED_GRIPPER, abs=1e-3)
    # observation reports the same joint-space values back
    obs = r.get_observation()
    assert obs["gripper.pos"] == pytest.approx(-0.045, abs=1e-4)
    assert obs["base.pos"] == pytest.approx(JOINT_LIMITS["base"][1], abs=1e-3)


def test_send_action_without_rate_limit(tmp_path, patch_can, fake_cameras):
    fake = patch_can(FakeCanBus(make_fake_motors(FOLLOWER_MOTOR_MODELS, FOLLOWER_CAN_IDS)))
    cfg = S1FollowerConfig(
        id="nolimit", calibration_dir=tmp_path, can_interface="virtual", max_relative_target=None
    )
    r = S1Follower(cfg)
    write_calibration(r)
    r = S1Follower(cfg)
    r.connect()
    sent = r.send_action({"link2.pos": 2.0, "unknown.pos": 1.0})
    assert sent == {"link2.pos": pytest.approx(2.0)}
    assert fake.motors["link2"].position == pytest.approx(2.0, abs=1e-3)
    r.disconnect()


def test_range_guard_refuses_out_of_range_start(tmp_path, patch_can, fake_cameras):
    positions = {"link4": 2.5}  # far outside -0.65..0.75
    fake = patch_can(FakeCanBus(make_fake_motors(FOLLOWER_MOTOR_MODELS, FOLLOWER_CAN_IDS, positions)))
    cfg = S1FollowerConfig(id="guard", calibration_dir=tmp_path, can_interface="virtual")
    r = S1Follower(cfg)
    write_calibration(r)
    r = S1Follower(cfg)
    with pytest.raises(RuntimeError, match="link4=\\+2.500"):
        r.connect()
    assert not r.is_connected
    assert not fake.motors["link4"].enabled


def test_inverted_joint_config_is_written_to_calibration(tmp_path, patch_can, fake_cameras, monkeypatch):
    fake = patch_can(FakeCanBus(make_fake_motors(FOLLOWER_MOTOR_MODELS, FOLLOWER_CAN_IDS)))
    cfg = S1FollowerConfig(
        id="inv", calibration_dir=tmp_path, can_interface="virtual", inverted_joints=["base"]
    )
    r = S1Follower(cfg)
    monkeypatch.setattr("builtins.input", lambda *_: "")
    # emulate the range-recording sweep: motors move while _record_ranges polls
    sweep = iter(range(1000))

    def moving_read(*args, **kwargs):
        i = next(sweep)
        for j in JOINTS:
            lo, hi = (CLOSED_GRIPPER, 0.0) if j == "gripper" else JOINT_LIMITS[j]
            fake.motors[j].position = lo if i % 2 else hi
        return orig_read(*args, **kwargs)

    orig_read = r.bus.read_positions
    monkeypatch.setattr(r.bus, "read_positions", moving_read)
    presses = iter([False, False, True])
    monkeypatch.setattr(follower_mod, "enter_pressed", lambda: next(presses))
    monkeypatch.setattr(follower_mod, "move_cursor_up", lambda n: None)
    r.connect(calibrate=False)
    r.calibrate()
    assert r.calibration["base"].drive_mode == 1 and r.calibration["link1"].drive_mode == 0
    assert r.calibration["gripper"].range_max == 0 and r.calibration["gripper"].range_min < 0
    assert all(fake.motors[j].zero_calls == 1 for j in JOINTS)
    assert r.calibration_fpath.is_file()
    r.disconnect()
