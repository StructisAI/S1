# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Structis S1A follower arm as a LeRobot ``Robot``."""

from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.utils import enter_pressed, move_cursor_up

from .calibration import JointMap, joint_map_from_calibration, make_calibration
from .config_s1_follower import S1FollowerConfig
from .constants import FOLLOWER_MASTER_ID_OFFSET, FOLLOWER_MOTOR_MODELS, GRIPPER, JOINTS, clamp
from .motors import DamiaoBus, DamiaoMotor

logger = logging.getLogger(__name__)


class S1Follower(Robot):
    """S1A: 6 Damiao joints + parallel gripper on CAN, wrist camera over USB.

    Observation: ``{<joint>.pos: float, ...}`` (rad, gripper in m) plus one image per camera.
    Action: ``{<joint>.pos: float, ...}`` in the same units. Targets are clamped to the calibrated
    range and rate-limited by ``max_relative_target`` before being sent as MIT position holds.
    """

    config_class = S1FollowerConfig
    name = "s1_follower"

    def __init__(self, config: S1FollowerConfig):
        super().__init__(config)
        self.config = config
        motors = {}
        for joint in JOINTS:
            kp, kd = config.mit_gains[joint]
            can_id = config.can_ids[joint]
            motors[joint] = DamiaoMotor(
                can_id=can_id,
                master_id=can_id + FOLLOWER_MASTER_ID_OFFSET,
                model=FOLLOWER_MOTOR_MODELS[joint],
                kp=kp,
                kd=kd,
                torque_limit=config.torque_limits.get(joint),
            )
        self.bus = DamiaoBus(
            motors, interface=config.can_interface, channel=config.can_channel, bitrate=config.can_bitrate
        )
        self.cameras = make_cameras_from_configs(config.cameras)
        self._maps: dict[str, JointMap] = {}
        self._last_target: dict[str, float] = {}
        self._load_maps()

    # ------------------------------------------------------------------ features
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in JOINTS}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (cfg.height, cfg.width, 3) for cam, cfg in self.config.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # ------------------------------------------------------------------ connection
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")
        self.bus.connect()
        try:
            for cam in self.cameras.values():
                cam.connect()
            if not self.is_calibrated and calibrate:
                self.calibrate()
            if self.is_calibrated:
                self.configure()
            else:
                logger.warning("%s is not calibrated: motors stay torque-free. Run calibrate().", self)
        except Exception:
            self._close_all()
            raise
        logger.info("%s connected.", self)

    def disconnect(self) -> None:
        if not self.bus.is_connected and not any(cam.is_connected for cam in self.cameras.values()):
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self._close_all()
        logger.info("%s disconnected.", self)

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected. Run `.connect()` first.")

    def _close_all(self) -> None:
        self.bus.disconnect(disable_torque=self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()

    # ------------------------------------------------------------------ calibration
    def _load_maps(self) -> None:
        self._maps = {}
        if not all(joint in self.calibration for joint in JOINTS):
            return
        try:
            self._maps = {
                joint: joint_map_from_calibration(joint, self.calibration[joint]) for joint in JOINTS
            }
        except ValueError as exc:
            logger.warning("Ignoring invalid calibration %s: %s", self.calibration_fpath, exc)

    @property
    def is_calibrated(self) -> bool:
        return len(self._maps) == len(JOINTS)

    def calibrate(self) -> None:
        """Interactive: zero pose, then range of motion. Saves to ``calibration_fpath``."""
        if self.calibration and self.is_calibrated:
            answer = input(
                f"Press ENTER to keep the calibration file for id {self.id}, "
                "or type 'c' and ENTER to recalibrate: "
            )
            if answer.strip().lower() != "c":
                return
        logger.info("Running calibration of %s", self)
        print("\nReleasing torque on the S1A: support the arm, it will move freely from now on.")
        self.bus.disable_torque()
        self.bus.enable_limp()
        input(
            "\nMove the S1A to its ZERO POSE (see the assembly instructions: every joint at the URDF zero, "
            "gripper fully OPEN) and press ENTER..."
        )
        # Make the zero pose the motors' own zero, so the one-turn absolute encoders wrap as far
        # away from the working range as possible after a power cycle.
        self.bus.disable_torque()
        self.bus.set_zero_position()
        self.bus.enable_limp()
        zero = self.bus.read_positions()

        print("\nMove EVERY joint slowly through its full range of motion, including the gripper.")
        print("Positions are recorded live. Press ENTER when done...")
        mins, maxes = self._record_ranges()

        self.calibration = {
            joint: make_calibration(
                joint,
                self.config.can_ids[joint],
                joint in self.config.inverted_joints,
                zero[joint],
                mins[joint],
                maxes[joint],
            )
            for joint in JOINTS
        }
        self._save_calibration()
        self._load_maps()
        logger.info("Calibration saved to %s", self.calibration_fpath)

    def _record_ranges(self) -> tuple[dict[str, float], dict[str, float]]:
        pos = self.bus.read_positions()
        mins, maxes = dict(pos), dict(pos)
        while True:
            pos = self.bus.read_positions()
            mins = {j: min(mins[j], pos[j]) for j in JOINTS}
            maxes = {j: max(maxes[j], pos[j]) for j in JOINTS}
            print("\n-------------------------------------------")
            print(f"{'JOINT':<10} | {'MIN':>8} | {'POS':>8} | {'MAX':>8}   (motor rad)")
            for j in JOINTS:
                print(f"{j:<10} | {mins[j]:>8.3f} | {pos[j]:>8.3f} | {maxes[j]:>8.3f}")
            if enter_pressed():
                return mins, maxes
            move_cursor_up(len(JOINTS) + 3)
            time.sleep(0.05)

    # ------------------------------------------------------------------ configure
    def configure(self) -> None:
        """Enable torque: every joint holds its present position. Refuses out-of-range joints."""
        if not self.is_calibrated:
            raise RuntimeError(f"{self} is not calibrated. Run `lerobot-calibrate` or calibrate() first.")
        present = self._read_joints()
        margin = self.config.range_guard_margin
        bad = []
        for j in JOINTS:
            q, m = present[j], self._maps[j]
            if q < m.low - margin or q > m.high + margin:
                bad.append(f"{j}={q:+.3f} (range {m.low:+.3f}..{m.high:+.3f})")
        if bad:
            self.bus.disable_torque()
            raise RuntimeError(
                "Refusing to enable torque, joints read outside their calibrated range: "
                + ", ".join(bad)
                + ". Power the arm on in the rest pose, or recalibrate."
            )
        self.bus.enable_torque()
        self._last_target = {j: self._maps[j].clamp(present[j]) for j in JOINTS}

    # ------------------------------------------------------------------ I/O
    def _read_joints(self) -> dict[str, float]:
        motor_pos = self.bus.read_positions()
        return {j: self._maps[j].motor_to_joint(motor_pos[j]) for j in JOINTS}

    def get_observation(self) -> dict[str, Any]:
        self._require_connected()
        start = time.perf_counter()
        obs: dict[str, Any] = {f"{j}.pos": q for j, q in self._read_joints().items()}
        logger.debug("%s read state: %.1fms", self, (time.perf_counter() - start) * 1e3)
        for name, cam in self.cameras.items():
            start = time.perf_counter()
            obs[name] = cam.async_read()
            logger.debug("%s read %s: %.1fms", self, name, (time.perf_counter() - start) * 1e3)
        return obs

    def _safe_targets(self, action: dict[str, Any]) -> dict[str, float]:
        """Clamp requested joint values to the calibrated range and to the per-step rate limit."""
        targets: dict[str, float] = {}
        for joint in JOINTS:
            key = f"{joint}.pos"
            if key not in action:
                continue
            goal = self._maps[joint].clamp(float(action[key]))
            step = (
                self.config.gripper_max_relative_target
                if joint == GRIPPER
                else self.config.max_relative_target
            )
            prev = self._last_target.get(joint)
            if step is not None and prev is not None:
                goal = clamp(goal, prev - step, prev + step)
            targets[joint] = goal
        return targets

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        self._require_connected()
        if not self.bus.is_holding(JOINTS[0]):
            raise RuntimeError(f"{self} torque is not enabled; call configure() first.")
        targets = self._safe_targets(action)
        self.bus.write_positions({j: self._maps[j].joint_to_motor(q) for j, q in targets.items()})
        self._last_target.update(targets)
        return {f"{j}.pos": q for j, q in targets.items()}
