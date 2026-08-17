# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""LeRobot plugin for the Structis S1 kit: S1A follower (``s1_follower``) and S1L leader (``s1_leader``)."""

from .config_s1_follower import S1FollowerConfig, default_wrist_camera
from .config_s1_leader import S1LeaderConfig
from .constants import JOINT_LIMITS, JOINTS
from .s1_follower import S1Follower
from .s1_leader import S1Leader

__version__ = "0.1.0"

__all__ = [
    "JOINTS",
    "JOINT_LIMITS",
    "S1Follower",
    "S1FollowerConfig",
    "S1Leader",
    "S1LeaderConfig",
    "default_wrist_camera",
]
