# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Configuration for the Structis S1L leader arm."""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.teleoperators import TeleoperatorConfig

from .constants import LEADER_MOTOR_MODEL


@TeleoperatorConfig.register_subclass("s1_leader")
@dataclass
class S1LeaderConfig(TeleoperatorConfig):
    """S1L: passive replica of the S1A with one Dynamixel XL330 per joint over USB.

    ``get_action`` returns the follower's action space (``<joint>.pos`` in rad, gripper in m,
    0 = open) by mapping each joint's calibrated range onto the URDF limits.
    """

    # Serial port of the U2D2 / USB adapter, e.g. /dev/ttyUSB0, /dev/tty.usbserial-XXXX, COM4.
    port: str
    motor_model: str = LEADER_MOTOR_MODEL

    # Joints whose leader motor turns opposite to the follower's URDF axis (stored as drive_mode).
    inverted_joints: list[str] = field(default_factory=list)

    # Hold the gripper trigger open with a small current so it springs back when released.
    gripper_spring: bool = True
    gripper_spring_current: int = 80  # Goal_Current units (~mA on XL330)
