# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Motor <-> URDF mapping and calibration file round trip."""

import io

import draccus
import pytest
from lerobot.motors import MotorCalibration

from lerobot_robot_s1.calibration import joint_map_from_calibration, make_calibration
from lerobot_robot_s1.constants import GRIPPER_TRAVEL_M, JOINT_LIMITS, JOINTS


def test_revolute_joint_roundtrip():
    cal = make_calibration("link1", 2, inverted=False, zero_rad=0.4, motor_min_rad=-1.5, motor_max_rad=2.3)
    jm = joint_map_from_calibration("link1", cal)
    assert jm.motor_to_joint(0.4) == pytest.approx(0.0, abs=1e-3)
    assert jm.motor_to_joint(1.4) == pytest.approx(1.0, abs=1e-3)
    assert jm.joint_to_motor(jm.motor_to_joint(0.9)) == pytest.approx(0.9, abs=1e-6)
    # recorded range intersected with the URDF limits
    assert jm.low == pytest.approx(max(-1.9, JOINT_LIMITS["link1"][0]), abs=1e-3)
    assert jm.high == pytest.approx(min(1.9, JOINT_LIMITS["link1"][1]), abs=1e-3)


def test_inverted_joint_flips_sign():
    cal = make_calibration("base", 1, inverted=True, zero_rad=0.0, motor_min_rad=-1.0, motor_max_rad=1.0)
    assert cal.drive_mode == 1
    jm = joint_map_from_calibration("base", cal)
    assert jm.motor_to_joint(0.5) == pytest.approx(-0.5, abs=1e-3)
    assert jm.joint_to_motor(-0.5) == pytest.approx(0.5, abs=1e-3)


@pytest.mark.parametrize("closing_positive", [True, False])
def test_gripper_maps_open_to_zero_and_closed_to_minus_travel(closing_positive):
    zero = 0.2  # open
    closed = zero + (5.0 if closing_positive else -5.0)
    cal = make_calibration(
        "gripper",
        7,
        inverted=False,
        zero_rad=zero,
        motor_min_rad=min(zero, closed),
        motor_max_rad=max(zero, closed),
    )
    assert cal.range_max == 0 and cal.range_min < 0
    jm = joint_map_from_calibration("gripper", cal)
    assert jm.motor_to_joint(zero) == pytest.approx(0.0, abs=1e-5)
    assert jm.motor_to_joint(closed) == pytest.approx(-GRIPPER_TRAVEL_M, abs=1e-5)
    assert jm.joint_to_motor(-GRIPPER_TRAVEL_M / 2) == pytest.approx((zero + closed) / 2, abs=1e-3)
    assert (jm.low, jm.high) == JOINT_LIMITS["gripper"]


def test_unmoved_joint_is_rejected():
    with pytest.raises(ValueError, match="not moved"):
        make_calibration("link2", 3, False, 0.0, 0.0, 0.01)
    with pytest.raises(ValueError, match="Gripper was not moved"):
        make_calibration("gripper", 7, False, 0.0, -0.01, 0.01)


def test_calibration_survives_lerobot_json_format():
    cal = {
        j: make_calibration(
            j, i + 1, inverted=(j == "base"), zero_rad=0.1, motor_min_rad=-1.0, motor_max_rad=1.5
        )
        for i, j in enumerate(JOINTS)
    }
    buf = io.StringIO()
    with draccus.config_type("json"):
        draccus.dump(cal, buf, indent=2)
    buf.seek(0)
    with draccus.config_type("json"):
        loaded = draccus.load(dict[str, MotorCalibration], buf)
    assert loaded == cal
    for j in JOINTS:
        a, b = joint_map_from_calibration(j, cal[j]), joint_map_from_calibration(j, loaded[j])
        assert a == b
