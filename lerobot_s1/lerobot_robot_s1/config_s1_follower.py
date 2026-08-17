# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Configuration for the Structis S1A follower arm."""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig

from .constants import FOLLOWER_CAN_IDS, FOLLOWER_MIT_GAINS, FOLLOWER_TORQUE_LIMITS_NM


def default_wrist_camera(index_or_path: int | str = 0) -> OpenCVCameraConfig:
    """Wrist camera (global-shutter UVC) defaults. Find the index with ``lerobot-find-cameras opencv``."""
    return OpenCVCameraConfig(index_or_path=index_or_path, fps=30, width=640, height=480)


@RobotConfig.register_subclass("s1_follower")
@dataclass
class S1FollowerConfig(RobotConfig):
    """S1A: 6 revolute Damiao joints + 1 prismatic gripper on one CAN bus.

    Actions and observations use URDF units: rad for ``base..link5``, metres for ``gripper``
    (0 = open, -0.045 = closed). Keys are ``<joint>.pos``.
    """

    # python-can settings. Linux/SocketCAN: interface="socketcan", channel="can0".
    # macOS/Windows with a CANable/candleLight adapter: interface="gs_usb", channel="0".
    # Serial (slcan) adapters: interface="slcan", channel="/dev/tty.usbmodemXXXX" or "COM5".
    can_interface: str = "socketcan"
    can_channel: str = "can0"
    can_bitrate: int = 1_000_000

    # CAN id per joint (status frames arrive on id + 0x10).
    can_ids: dict[str, int] = field(default_factory=lambda: dict(FOLLOWER_CAN_IDS))

    # MIT gains (kp, kd) per joint and measured-torque limits (Nm) per joint.
    mit_gains: dict[str, tuple[float, float]] = field(default_factory=lambda: dict(FOLLOWER_MIT_GAINS))
    torque_limits: dict[str, float] = field(default_factory=lambda: dict(FOLLOWER_TORQUE_LIMITS_NM))

    # Largest step a target may take per send_action(), relative to the previous target
    # (rad for joints, m for the gripper). Bounds the commanded speed; None disables the clamp.
    max_relative_target: float | None = 0.05
    gripper_max_relative_target: float | None = 0.005

    # Joints whose motor turns opposite to the URDF axis. Written to the calibration file
    # (drive_mode) when calibrating. The mirrored left arm typically needs a different set.
    inverted_joints: list[str] = field(default_factory=list)

    # Refuse to enable torque if a joint reads this far (rad) outside its calibrated range.
    # Damiao encoders are absolute over one turn: power the arm on in the rest pose.
    range_guard_margin: float = 0.2

    disable_torque_on_disconnect: bool = True

    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {"wrist": default_wrist_camera(0)})
