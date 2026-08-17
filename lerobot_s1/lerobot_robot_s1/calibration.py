# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Mapping between S1A motor positions and URDF joint values.

The follower's calibration file (LeRobot format, ``dict[str, MotorCalibration]``) stores integers
in Damiao position codes (see :mod:`.constants`):

* ``homing_offset``: added to ``drive_mode``-signed motor position so the URDF zero pose reads 0.
* ``drive_mode``: 1 if the motor turns against the URDF axis (value is negated).
* ``range_min`` / ``range_max``: recorded range of motion in the same (zero-shifted, signed) space.
  For the gripper the closed position is ``range_min`` (< 0) and open is ``range_max`` (= 0);
  the metre scale is derived from it.
"""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.motors import MotorCalibration

from .constants import GRIPPER, GRIPPER_TRAVEL_M, JOINT_LIMITS, clamp, code_to_rad, rad_to_code

MIN_RANGE_RAD = 0.05


@dataclass(frozen=True)
class JointMap:
    """Bidirectional map for one joint: motor rad <-> URDF value (rad, or m for the gripper)."""

    sign: float
    offset_rad: float
    scale: float  # URDF units per rad of zero-shifted motor travel (1.0 for revolute joints)
    low: float  # URDF units
    high: float

    def motor_to_joint(self, motor_rad: float) -> float:
        return (self.sign * motor_rad + self.offset_rad) * self.scale

    def joint_to_motor(self, value: float) -> float:
        return self.sign * (value / self.scale - self.offset_rad)

    def clamp(self, value: float) -> float:
        return clamp(value, self.low, self.high)


def joint_map_from_calibration(name: str, cal: MotorCalibration) -> JointMap:
    sign = -1.0 if cal.drive_mode else 1.0
    offset = code_to_rad(cal.homing_offset)
    urdf_low, urdf_high = JOINT_LIMITS[name]
    if name == GRIPPER:
        closed_rel = code_to_rad(cal.range_min)
        if closed_rel >= -MIN_RANGE_RAD:
            raise ValueError(f"Invalid gripper calibration: range_min={cal.range_min} must be < 0")
        scale = GRIPPER_TRAVEL_M / -closed_rel
        return JointMap(sign, offset, scale, urdf_low, urdf_high)
    low = max(code_to_rad(cal.range_min), urdf_low)
    high = min(code_to_rad(cal.range_max), urdf_high)
    if high - low < MIN_RANGE_RAD:
        raise ValueError(f"Invalid calibration for {name!r}: range [{low:.3f}, {high:.3f}] rad")
    return JointMap(sign, offset, 1.0, low, high)


def make_calibration(
    name: str,
    motor_id: int,
    inverted: bool,
    zero_rad: float,
    motor_min_rad: float,
    motor_max_rad: float,
) -> MotorCalibration:
    """Build a calibration entry from a recorded zero pose and raw motor extremes.

    ``inverted`` is ignored for the gripper: its sign is chosen so that closing is negative, from
    which side of the open (zero) position the recorded travel lies.
    """
    if name == GRIPPER:
        ext_pos = motor_max_rad - zero_rad
        ext_neg = zero_rad - motor_min_rad
        travel = max(ext_pos, ext_neg)
        if travel < MIN_RANGE_RAD:
            raise ValueError("Gripper was not moved during range recording")
        sign = -1.0 if ext_pos >= ext_neg else 1.0
        offset = -sign * zero_rad
        return MotorCalibration(
            id=motor_id,
            drive_mode=1 if sign < 0 else 0,
            homing_offset=rad_to_code(offset),
            range_min=rad_to_code(-travel),
            range_max=0,
        )
    sign = -1.0 if inverted else 1.0
    offset = -sign * zero_rad
    a = sign * motor_min_rad + offset
    b = sign * motor_max_rad + offset
    low, high = min(a, b), max(a, b)
    if high - low < MIN_RANGE_RAD:
        raise ValueError(f"Joint {name!r} was not moved during range recording")
    return MotorCalibration(
        id=motor_id,
        drive_mode=1 if inverted else 0,
        homing_offset=rad_to_code(offset),
        range_min=rad_to_code(low),
        range_max=rad_to_code(high),
    )
