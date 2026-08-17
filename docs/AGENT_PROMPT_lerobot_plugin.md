# Task: build the public LeRobot plugin for the Structis S1 kit

You are working inside the Structis production codebase. Your job is to extract and
package a clean, public, standalone LeRobot plugin for the **S1 kit** into a new folder
`lerobot_s1/` (to be copied into the public repo `github.com/StructisAI/S1`). Nothing
outside that folder is public. Do not move or delete production code.

## What the S1 kit is
- **S1A follower**: 6 revolute axes + 1 prismatic parallel gripper. Damiao CAN-bus motors
  (base J8009P; mid-arm J4340P; wrist + gripper J4310P), CAN-USB adapter to the host.
  Global-shutter wide-angle wrist camera (USB/UVC). 4 kg payload, 700 mm reach.
- **S1L leader**: passive teleop arm, Dynamixel XL330 on every joint incl. gripper, USB.
- Joint names and order (must match the public URDF `s1_description`):
  `base, link1, link2, link3, link4, link5, gripper`.
  Limits (rad / m): base −1.59..1.57 · link1 −1.98..1.97 · link2 −1.18..3.96 ·
  link3 −2.12..1.95 · link4 −0.65..0.75 · link5 −3.17..3.08 · gripper −0.045..0
  (0 = open, −0.045 = closed, 45 mm per finger). Left arm is a mirrored variant.
- Host: macOS / Windows / Linux, Python ≥ 3.10.

## Deliverables (all inside `lerobot_s1/`)
1. **A pip-installable package** that LeRobot discovers as a plugin.
   First, inspect the *installed* LeRobot version and its docs/source for the current
   plugin mechanism (package naming convention, entry points, `@RobotConfig.register_subclass`
   / `@TeleoperatorConfig.register_subclass`, `Robot` / `Teleoperator` base classes, camera
   config classes). Follow exactly what the installed version expects — do not guess from memory.
   Target the LeRobot version pinned in this repo; note it in `pyproject.toml`.
2. **`S1FollowerConfig` + `S1Follower(Robot)`** — `connect`, `calibrate`, `configure`,
   `get_observation` (joint positions + wrist camera), `send_action`, `disconnect`,
   `is_connected`, `is_calibrated`, `observation_features`, `action_features`.
   Motor bus: reuse or extract our Damiao CAN driver as a small self-contained module
   (`motors/damiao.py`) — only what the plugin needs; no cell / RL / dataset code.
   Include torque limits, a safe max-relative-target clamp, and a clean disconnect that
   disables torque.
3. **`S1LeaderConfig` + `S1Leader(Teleoperator)`** — reads XL330 positions via LeRobot's
   Dynamixel bus, `get_action` in the follower's action space, calibration.
4. **Calibration**: LeRobot's standard calibration flow and file format
   (`~/.cache/huggingface/lerobot/calibration/...`), homing offsets + range for every joint;
   gripper mapped 0 = open.
5. **Camera**: default config for the wrist camera via LeRobot's OpenCV camera class,
   with sensible fps/resolution defaults; document how to find the index.
6. **CLI examples that actually run** (put in `README.md`): `lerobot-find-port`,
   `lerobot-calibrate`, `lerobot-teleoperate`, `lerobot-record`, `lerobot-replay`,
   `lerobot-train --policy.type=act`, and one `--policy.type=diffusion` example.
   Also a `examples/teleop_minimal.py` that uses the classes directly.
7. **Bimanual**: document how to run two follower/leader pairs (ports, left/right
   naming) — a config example is enough if LeRobot supports multi-robot configs; otherwise
   a short script.
8. **Tests**: `pytest` unit tests with the motor bus and camera mocked (connect/disconnect,
   action clamping, calibration round-trip, feature dicts). No hardware needed in CI.
9. **Packaging**: `pyproject.toml` (name, version 0.1.0, Apache-2.0, deps: lerobot pinned,
   python-can or whatever the Damiao driver needs), `README.md`, `CHANGELOG.md`, `LICENSE`
   (Apache 2.0). Lint clean (`ruff`), typed where cheap.
10. **Safety notes in the README** (short): no e-stop in the arm, keep out of the reach
    envelope while powered, start with reduced speed/torque limits, see the assembly
    instructions shipped with the kit.

## Constraints
- Public repo: **no** CAD, BOM, supplier info, internal URLs, credentials, cell software,
  RL/training internals, customer data. Grep the folder for `nextis`, `aira`, internal
  hostnames and remove/rename before finishing (public naming is **Structis S1 / S1A / S1L**).
- Keep it small: a reader should understand the plugin in 20 minutes.
- Do not break the production package; the plugin must import only from itself + lerobot
  + its declared deps.
- If something in production is genuinely not extractable (e.g. the CAN driver is tangled),
  write a minimal clean-room version for the plugin and say so.

## Hand-off
Finish with a short report: LeRobot version targeted, plugin discovery mechanism used,
files created, test results, anything you could not verify without hardware, and the
exact commands a customer runs from unboxing to first recorded episode.
