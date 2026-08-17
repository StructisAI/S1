# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Structis S1L leader arm as a LeRobot ``Teleoperator``."""

from __future__ import annotations

import logging
import time

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode
from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_s1_leader import S1LeaderConfig
from .constants import ARM_JOINTS, GRIPPER, GRIPPER_TRAVEL_M, JOINT_LIMITS, JOINTS, LEADER_MOTOR_IDS

logger = logging.getLogger(__name__)

GRIPPER_SPRING_MARGIN_TICKS = 100  # keep the spring target off the mechanical end stop


def leader_to_joint(name: str, normalized: float, inverted: bool) -> float:
    """Map a LeRobot-normalized leader reading onto URDF units.

    Arm joints arrive in [-100, 100] (range_min..range_max), the gripper in [0, 100].
    """
    low, high = JOINT_LIMITS[name]
    if name == GRIPPER:
        closed_frac = (100.0 - normalized if inverted else normalized) / 100.0
        return -GRIPPER_TRAVEL_M * min(1.0, max(0.0, closed_frac))
    n = -normalized if inverted else normalized
    frac = (n + 100.0) / 200.0
    return low + min(1.0, max(0.0, frac)) * (high - low)


class S1Leader(Teleoperator):
    """S1L leader: reads XL330 positions and emits S1A joint targets."""

    config_class = S1LeaderConfig
    name = "s1_leader"

    def __init__(self, config: S1LeaderConfig):
        super().__init__(config)
        self.config = config
        motors = {
            joint: Motor(
                LEADER_MOTOR_IDS[joint],
                config.motor_model,
                MotorNormMode.RANGE_0_100 if joint == GRIPPER else MotorNormMode.RANGE_M100_100,
            )
            for joint in JOINTS
        }
        self.bus = DynamixelMotorsBus(port=config.port, motors=motors, calibration=self.calibration)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in JOINTS}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info("No matching calibration for %s, running calibration", self)
            self.calibrate()
        self.configure()
        logger.info("%s connected.", self)

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        """LeRobot flow: mid-range homing, range of motion, then the open gripper position."""
        self.bus.disable_torque()
        if self.calibration:
            answer = input(
                f"Press ENTER to write the calibration file for id {self.id} to the motors, "
                "or type 'c' and ENTER to recalibrate: "
            )
            if answer.strip().lower() != "c":
                self.bus.write_calibration(self.calibration)
                return
        logger.info("Running calibration of %s", self)
        for joint in JOINTS:
            self.bus.write("Operating_Mode", joint, OperatingMode.EXTENDED_POSITION.value)

        input("\nMove the S1L to the MIDDLE of its range of motion on every joint and press ENTER...")
        homing_offsets = self.bus.set_half_turn_homings()

        print("\nMove EVERY joint through its full range of motion, and squeeze the gripper trigger fully.")
        print("Positions are recorded live. Press ENTER when done...")
        range_mins, range_maxes = self.bus.record_ranges_of_motion()

        input("\nRelease the gripper trigger (fully OPEN) and press ENTER...")
        gripper_open = self.bus.read("Present_Position", GRIPPER, normalize=False)
        gripper_inverted = abs(gripper_open - range_maxes[GRIPPER]) < abs(gripper_open - range_mins[GRIPPER])

        self.calibration = {}
        for joint, motor in self.bus.motors.items():
            inverted = gripper_inverted if joint == GRIPPER else joint in self.config.inverted_joints
            self.calibration[joint] = MotorCalibration(
                id=motor.id,
                drive_mode=1 if inverted else 0,
                homing_offset=homing_offsets[joint],
                range_min=range_mins[joint],
                range_max=range_maxes[joint],
            )
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info("Calibration saved to %s", self.calibration_fpath)

    def configure(self) -> None:
        """Torque off on the arm joints; optional current-limited spring on the gripper trigger."""
        self.bus.disable_torque()
        self.bus.configure_motors()
        for joint in ARM_JOINTS:
            self.bus.write("Operating_Mode", joint, OperatingMode.EXTENDED_POSITION.value)
        self.bus.write("Operating_Mode", GRIPPER, OperatingMode.CURRENT_POSITION.value)
        if self.config.gripper_spring and self.is_calibrated:
            cal = self.calibration[GRIPPER]
            if cal.drive_mode:
                open_ticks = cal.range_max - GRIPPER_SPRING_MARGIN_TICKS
            else:
                open_ticks = cal.range_min + GRIPPER_SPRING_MARGIN_TICKS
            self.bus.write("Goal_Current", GRIPPER, self.config.gripper_spring_current, normalize=False)
            self.bus.enable_torque(GRIPPER)
            self.bus.write("Goal_Position", GRIPPER, open_ticks, normalize=False)

    def setup_motors(self) -> None:
        for joint in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{joint}' motor only and press ENTER.")
            self.bus.setup_motor(joint)
            print(f"'{joint}' motor id set to {self.bus.motors[joint].id}")

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected. Run `.connect()` first.")
        start = time.perf_counter()
        raw = self.bus.sync_read("Present_Position")
        action = {
            f"{joint}.pos": leader_to_joint(joint, raw[joint], bool(self.calibration[joint].drive_mode))
            for joint in JOINTS
        }
        logger.debug("%s read action: %.1fms", self, (time.perf_counter() - start) * 1e3)
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.bus.disconnect()
        logger.info("%s disconnected.", self)
