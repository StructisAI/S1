# Changelog

All notable changes to `lerobot_robot_s1` are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.1.0] - 2026-08-17

### Added
- `S1Follower` / `S1FollowerConfig` (`--robot.type=s1_follower`): S1A arm on Damiao CAN motors,
  wrist camera via LeRobot's OpenCV camera, URDF-unit action/observation space, calibrated range
  clamp, per-step rate limit, measured-torque yield, torque-off on disconnect.
- `S1Leader` / `S1LeaderConfig` (`--teleop.type=s1_leader`): S1L arm on Dynamixel XL330 via
  LeRobot's Dynamixel bus, LeRobot calibration flow, gripper trigger spring.
- Clean-room Damiao MIT-mode driver on python-can (`lerobot_robot_s1.motors.damiao`).
- Examples: `examples/teleop_minimal.py`, `examples/bimanual_teleop.py`.
- Hardware-free test suite (fake CAN bus, fake Dynamixel bus, fake camera).
