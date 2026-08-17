# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Shared constants for the Structis S1 kit (S1A follower, S1L leader).

Joint names, order and limits match the public URDF ``s1_description``.
"""

from __future__ import annotations

import math

# Joint order is the action/observation order and matches the URDF.
JOINTS: tuple[str, ...] = ("base", "link1", "link2", "link3", "link4", "link5", "gripper")
ARM_JOINTS: tuple[str, ...] = JOINTS[:-1]
GRIPPER = "gripper"

# URDF limits. Revolute joints in rad, gripper (prismatic) in m: 0 = open, -0.045 = closed.
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "base": (-1.59, 1.57),
    "link1": (-1.98, 1.97),
    "link2": (-1.18, 3.96),
    "link3": (-2.12, 1.95),
    "link4": (-0.65, 0.75),
    "link5": (-3.17, 3.08),
    "gripper": (-0.045, 0.0),
}
GRIPPER_TRAVEL_M = 0.045  # per finger, 0 (open) -> -0.045 (closed)

# S1A motor layout: Damiao model + CAN id per joint. Status frames come back on id + 0x10.
FOLLOWER_MOTOR_MODELS: dict[str, str] = {
    "base": "J8009P",
    "link1": "J8009P",
    "link2": "J4340P",
    "link3": "J4340P",
    "link4": "J4310",
    "link5": "J4310",
    "gripper": "J4310",
}
FOLLOWER_CAN_IDS: dict[str, int] = {name: i + 1 for i, name in enumerate(JOINTS)}
FOLLOWER_MASTER_ID_OFFSET = 0x10

# Conservative MIT gains (kp, kd) per joint. Start here, raise once the arm is trusted.
FOLLOWER_MIT_GAINS: dict[str, tuple[float, float]] = {
    "base": (28.0, 1.2),
    "link1": (120.0, 5.0),
    "link2": (170.0, 4.5),
    "link3": (120.0, 4.0),
    "link4": (24.0, 0.2),  # J4310: keep kd low, higher kd saturates torque and oscillates
    "link5": (31.0, 0.2),
    "gripper": (15.0, 0.15),
}

# Measured-torque limits (Nm) per joint. Above this the joint yields (target snaps to the
# present position) until the load drops. Reduced values for first use.
FOLLOWER_TORQUE_LIMITS_NM: dict[str, float] = {
    "base": 14.0,
    "link1": 14.0,
    "link2": 12.0,
    "link3": 12.0,
    "link4": 4.0,
    "link5": 4.0,
    "gripper": 3.0,
}

# S1L: one Dynamixel XL330 per joint, ids 1..7 in joint order.
LEADER_MOTOR_IDS: dict[str, int] = {name: i + 1 for i, name in enumerate(JOINTS)}
LEADER_MOTOR_MODEL = "xl330-m077"

# Calibration files store positions as the Damiao 16-bit position code:
# 0 .. 65535  <->  -12.5 .. +12.5 rad. One code = 0.38 mrad.
DAMIAO_P_MAX = 12.5
DAMIAO_POS_CODES = (1 << 16) - 1
RAD_PER_CODE = 2 * DAMIAO_P_MAX / DAMIAO_POS_CODES


def rad_to_code(rad: float) -> int:
    return round(rad / RAD_PER_CODE)


def code_to_rad(code: int) -> float:
    return code * RAD_PER_CODE


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


TWO_PI = 2 * math.pi
