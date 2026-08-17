# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""S1Leader with the Dynamixel bus mocked."""

import pytest
from lerobot.motors import MotorCalibration

from lerobot_robot_s1 import S1Leader, S1LeaderConfig
from lerobot_robot_s1 import s1_leader as leader_mod
from lerobot_robot_s1.constants import GRIPPER_TRAVEL_M, JOINT_LIMITS, JOINTS, LEADER_MOTOR_IDS
from lerobot_robot_s1.s1_leader import leader_to_joint


@pytest.mark.parametrize("joint", JOINTS[:-1])
def test_arm_joint_mapping_hits_urdf_limits(joint):
    lo, hi = JOINT_LIMITS[joint]
    assert leader_to_joint(joint, -100.0, False) == pytest.approx(lo)
    assert leader_to_joint(joint, 100.0, False) == pytest.approx(hi)
    assert leader_to_joint(joint, 0.0, False) == pytest.approx((lo + hi) / 2)
    assert leader_to_joint(joint, 100.0, True) == pytest.approx(lo)
    assert leader_to_joint(joint, 250.0, False) == pytest.approx(hi)  # clamped


def test_gripper_mapping():
    assert leader_to_joint("gripper", 0.0, False) == pytest.approx(0.0)
    assert leader_to_joint("gripper", 100.0, False) == pytest.approx(-GRIPPER_TRAVEL_M)
    assert leader_to_joint("gripper", 100.0, True) == pytest.approx(0.0)
    assert leader_to_joint("gripper", 0.0, True) == pytest.approx(-GRIPPER_TRAVEL_M)
    assert leader_to_joint("gripper", 50.0, False) == pytest.approx(-GRIPPER_TRAVEL_M / 2)


class FakeDynamixelBus:
    def __init__(self, port, motors, calibration=None):
        self.port, self.motors, self.calibration = port, motors, calibration or {}
        self.is_connected = False
        self.writes: list[tuple] = []
        self.torque_enabled: set[str] = set()
        self.present = {m: 0.0 for m in motors}

    def connect(self):
        self.is_connected = True

    def disconnect(self, disable_torque=True):
        self.is_connected = False

    @property
    def is_calibrated(self):
        return bool(self.calibration)

    def write_calibration(self, cal, cache=True):
        self.calibration = cal

    def disable_torque(self, motors=None):
        self.torque_enabled.clear()

    def enable_torque(self, motors=None):
        self.torque_enabled.add(motors)

    def configure_motors(self):
        pass

    def write(self, name, motor, value, normalize=True, num_retry=0):
        self.writes.append((name, motor, value, normalize))

    def read(self, name, motor, normalize=True):
        return self.present[motor]

    def sync_read(self, name, motors=None, normalize=True):
        return dict(self.present)


@pytest.fixture
def leader(tmp_path, monkeypatch):
    monkeypatch.setattr(leader_mod, "DynamixelMotorsBus", FakeDynamixelBus)
    cfg = S1LeaderConfig(
        id="test_leader", port="/dev/fake", calibration_dir=tmp_path, inverted_joints=["link3"]
    )
    lead = S1Leader(cfg)
    lead.calibration = {
        j: MotorCalibration(
            id=LEADER_MOTOR_IDS[j],
            drive_mode=1 if j in ("link3", "gripper") else 0,
            homing_offset=0,
            range_min=1000,
            range_max=3000,
        )
        for j in JOINTS
    }
    lead._save_calibration()
    lead = S1Leader(cfg)  # reload from disk
    return lead


def test_leader_features_and_action(leader):
    assert list(leader.action_features) == [f"{j}.pos" for j in JOINTS]
    assert leader.feedback_features == {}
    leader.connect()
    assert leader.is_connected and leader.is_calibrated
    leader.bus.present.update({"base": 100.0, "link3": 100.0, "gripper": 100.0, "link1": 0.0})
    action = leader.get_action()
    assert set(action) == set(leader.action_features)
    assert action["base.pos"] == pytest.approx(JOINT_LIMITS["base"][1])
    assert action["link3.pos"] == pytest.approx(JOINT_LIMITS["link3"][0])  # inverted
    assert action["gripper.pos"] == pytest.approx(0.0)  # inverted gripper: 100 = open
    lo, hi = JOINT_LIMITS["link1"]
    assert action["link1.pos"] == pytest.approx((lo + hi) / 2)
    leader.disconnect()
    assert not leader.is_connected


def test_leader_configure_sets_gripper_spring(leader):
    leader.connect()
    ops = {(m, v) for name, m, v, _ in leader.bus.writes if name == "Operating_Mode"}
    assert ("gripper", 5) in ops and ("base", 4) in ops  # CURRENT_POSITION=5, EXTENDED_POSITION=4
    goal = [w for w in leader.bus.writes if w[0] == "Goal_Position"]
    assert goal and goal[-1][1] == "gripper" and goal[-1][3] is False
    assert goal[-1][2] == 3000 - leader_mod.GRIPPER_SPRING_MARGIN_TICKS  # inverted -> open at range_max
    assert "gripper" in leader.bus.torque_enabled
    leader.disconnect()


def test_leader_calibration_flow(tmp_path, monkeypatch):
    monkeypatch.setattr(leader_mod, "DynamixelMotorsBus", FakeDynamixelBus)
    lead = S1Leader(S1LeaderConfig(id="fresh", port="/dev/fake", calibration_dir=tmp_path))
    bus = lead.bus
    bus.set_half_turn_homings = lambda: {j: 100 for j in JOINTS}
    bus.record_ranges_of_motion = lambda: ({j: 500 for j in JOINTS}, {j: 3500 for j in JOINTS})
    bus.present["gripper"] = 3400  # released trigger sits near range_max -> inverted gripper
    monkeypatch.setattr("builtins.input", lambda *_: "")
    lead.connect(calibrate=True)
    assert lead.calibration["gripper"].drive_mode == 1
    assert lead.calibration["base"].drive_mode == 0
    assert lead.calibration["base"].range_min == 500 and lead.calibration["base"].range_max == 3500
    assert lead.calibration_fpath.is_file()
    lead.disconnect()
