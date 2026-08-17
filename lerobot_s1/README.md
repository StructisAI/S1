# lerobot_robot_s1 — LeRobot plugin for the Structis S1 kit

[LeRobot](https://github.com/huggingface/lerobot) plugin for the **Structis S1** teleoperation kit:

| Device | LeRobot type | What it is |
|---|---|---|
| **S1A follower** | `--robot.type=s1_follower` | 6 revolute axes + 1 prismatic parallel gripper. Damiao CAN motors (J8009P base/shoulder, J4340P elbow, J4310 wrist + gripper), one CAN-USB adapter, a global-shutter wide-angle wrist camera (USB/UVC). 4 kg payload, 700 mm reach. |
| **S1L leader** | `--teleop.type=s1_leader` | Passive replica of the S1A with a Dynamixel XL330 on every joint (incl. the gripper trigger), one USB serial adapter. |

Joint names and order match the public URDF `s1_description`:
`base, link1, link2, link3, link4, link5, gripper`. Actions and observations are in **URDF units**
(rad for the six joints, metres for the gripper: `0` = open, `-0.045` = closed). Keys are `<joint>.pos`.

Runs on Linux, macOS and Windows with Python ≥ 3.10. Developed against LeRobot 0.4.3; the APIs
used are unchanged through LeRobot 0.6.x.

## Safety first

The S1A has **no emergency stop of its own**. Before powering it:

- Bolt the base down and keep everyone out of the 700 mm reach envelope while the arm is powered.
- Wire a mains switch or a power-supply cut-off you can reach without entering the envelope, and use it.
- Start with the reduced defaults in this plugin (`max_relative_target`, `torque_limits`,
  `mit_gains`) and raise them step by step once the arm behaves.
- Torque is disabled when the plugin disconnects (Ctrl-C included): the arm will sag under
  gravity. Park it on the table or hold it before quitting.
- Read the assembly and commissioning instructions shipped with the kit; they take precedence
  over this README.

## Install

```bash
pip install lerobot_robot_s1            # from PyPI, or from a checkout:
pip install -e ".[dev]"
```

This pulls `lerobot[dynamixel]` and `python-can`. LeRobot's CLIs (`lerobot-*`) auto-discover the
plugin: the distribution and import name start with `lerobot_robot_`, and importing it registers
both `s1_follower` and `s1_leader`.

### CAN adapter

The S1A talks CAN at 1 Mbit/s through the adapter in the kit. Pick the python-can interface for
your OS and pass it as `--robot.can_interface` / `--robot.can_channel`:

| OS | Adapter mode | `can_interface` | `can_channel` | Setup |
|---|---|---|---|---|
| Linux | SocketCAN (candleLight / gs_usb, PEAK, …) | `socketcan` (default) | `can0` (default) | `sudo ip link set can0 up type can bitrate 1000000 && sudo ip link set can0 txqueuelen 256` |
| macOS / Windows | candleLight / CANable firmware | `gs_usb` | `0` (device index) | `pip install "lerobot_robot_s1[usb]"`; Windows needs the WinUSB driver (Zadig) on the adapter |
| any | slcan (serial) firmware | `slcan` | `/dev/tty.usbmodemXXXX`, `COM5` | none |

Anything python-can supports works; see the
[python-can interface docs](https://python-can.readthedocs.io/en/stable/interfaces.html).

## From unboxing to the first episode

```bash
# 1. Ports and cameras
lerobot-find-port                       # unplug/replug the S1L USB adapter -> its port
lerobot-find-cameras opencv             # lists indices/paths; the wrist camera is a 640x480 UVC device

# 2. Calibrate the follower (interactive: zero pose, then move every joint through its range)
lerobot-calibrate --robot.type=s1_follower --robot.id=s1a_right \
    --robot.can_interface=socketcan --robot.can_channel=can0 --robot.cameras='{}'

# 3. Calibrate the leader (interactive: middle pose, range of motion, released trigger)
lerobot-calibrate --teleop.type=s1_leader --teleop.id=s1l_right --teleop.port=/dev/ttyUSB0

# 4. Teleoperate (add --display_data=true for a live view)
lerobot-teleoperate \
    --robot.type=s1_follower --robot.id=s1a_right --robot.can_channel=can0 \
    --robot.cameras='{"wrist": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
    --teleop.type=s1_leader --teleop.id=s1l_right --teleop.port=/dev/ttyUSB0 \
    --fps=30

# 5. Record a dataset (30 fps, 20 episodes; Ctrl-C ends early, right arrow ends an episode)
lerobot-record \
    --robot.type=s1_follower --robot.id=s1a_right --robot.can_channel=can0 \
    --robot.cameras='{"wrist": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
    --teleop.type=s1_leader --teleop.id=s1l_right --teleop.port=/dev/ttyUSB0 \
    --dataset.repo_id=<hf_user>/s1_pick_place --dataset.single_task="Pick the cube and place it in the bin" \
    --dataset.fps=30 --dataset.num_episodes=20 --dataset.episode_time_s=30 --dataset.reset_time_s=10 \
    --dataset.push_to_hub=false

# 6. Replay an episode on the arm (clear the workspace first)
lerobot-replay --robot.type=s1_follower --robot.id=s1a_right --robot.can_channel=can0 --robot.cameras='{}' \
    --dataset.repo_id=<hf_user>/s1_pick_place --dataset.episode=0

# 7. Train
lerobot-train --dataset.repo_id=<hf_user>/s1_pick_place --policy.type=act \
    --output_dir=outputs/train/s1_act --job_name=s1_act --policy.device=cuda \
    --policy.push_to_hub=false --wandb.enable=false

lerobot-train --dataset.repo_id=<hf_user>/s1_pick_place --policy.type=diffusion \
    --policy.crop_shape='[420, 560]' \
    --output_dir=outputs/train/s1_diffusion --job_name=s1_diffusion --policy.device=cuda \
    --policy.push_to_hub=false --wandb.enable=false

# 8. Run the policy on the arm (records an evaluation dataset at the same time)
lerobot-record \
    --robot.type=s1_follower --robot.id=s1a_right --robot.can_channel=can0 \
    --robot.cameras='{"wrist": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
    --policy.path=outputs/train/s1_act/checkpoints/last/pretrained_model \
    --dataset.repo_id=<hf_user>/eval_s1_act --dataset.single_task="Pick the cube and place it in the bin" \
    --dataset.num_episodes=5 --dataset.push_to_hub=false
```

Notes:

- `--robot.cameras='{}'` runs without a camera (calibration, replay). The default config opens
  camera index 0 at 640×480@30; on a laptop that is usually the built-in webcam, so pass the
  index from `lerobot-find-cameras`.
- Damiao encoders are absolute over **one turn**. Power the S1A on in the rest pose from the
  manual. If a joint reads outside its calibrated range at connect, the plugin refuses to enable
  torque and tells you which joint (`range_guard_margin`).
- Datasets store `observation.state` and `action` as the 7 joint values in URDF units, and
  `observation.images.wrist`.

## Configuration

`S1FollowerConfig` (`--robot.*`):

| Field | Default | Meaning |
|---|---|---|
| `can_interface`, `can_channel`, `can_bitrate` | `socketcan`, `can0`, `1000000` | python-can bus |
| `can_ids` | `base=1 … gripper=7` | CAN id per joint; status frames arrive on id + 0x10 |
| `mit_gains` | per joint, e.g. `link1: (120, 5.0)` | MIT (kp, kd) sent with every target |
| `torque_limits` | per joint, e.g. `link1: 14 Nm` | above this measured torque the joint yields (target held at present position) until the load drops |
| `max_relative_target` / `gripper_max_relative_target` | `0.05 rad` / `0.005 m` per `send_action` | rate limit relative to the previous target; `null` disables |
| `inverted_joints` | `[]` | joints whose motor turns against the URDF axis; written to the calibration file (`drive_mode`) |
| `range_guard_margin` | `0.2 rad` | refuse to enable torque if a joint starts this far outside its calibrated range |
| `disable_torque_on_disconnect` | `true` | |
| `cameras` | `{"wrist": opencv index 0, 640×480@30}` | any LeRobot camera config |

`S1LeaderConfig` (`--teleop.*`): `port` (required), `motor_model` (`xl330-m077`),
`inverted_joints`, `gripper_spring` (hold the trigger open with `gripper_spring_current`).

If a joint moves the wrong way in teleop, add it to `--teleop.inverted_joints` (leader) or
`--robot.inverted_joints` (follower) and recalibrate that device.

## Calibration files

Standard LeRobot location and format:
`~/.cache/huggingface/lerobot/calibration/robots/s1_follower/<id>.json` and
`.../teleoperators/s1_leader/<id>.json`, one `MotorCalibration` per joint
(`id, drive_mode, homing_offset, range_min, range_max`).

- **Follower**: values are Damiao 16-bit position codes (0…65535 ↔ −12.5…+12.5 rad).
  `homing_offset` shifts the recorded zero pose to 0, `drive_mode` flips the sign, `range_*` is
  the recorded range (intersected with the URDF limits at run time). For the gripper `range_max`
  is 0 (open) and `range_min` the closed position, from which the metre scale is derived. The
  calibration also stores the zero pose as the motors' own zero (Damiao `0xFE`).
- **Leader**: LeRobot's Dynamixel convention (`set_half_turn_homings` + range of motion in
  ticks). Each joint's range is mapped linearly onto the URDF limits; `drive_mode=1` inverts.
  The gripper's `drive_mode` is set from the released-trigger position so 0 = open.

## Bimanual

LeRobot 0.4.x has no generic multi-robot config, so run two independent pairs with distinct ids
and CAN channels (each arm has its own adapter):

```bash
python examples/bimanual_teleop.py \
    --right-leader-port /dev/ttyUSB0 --right-can can0 \
    --left-leader-port  /dev/ttyUSB1 --left-can  can1
```

Calibrate each of the four devices with its own id (`s1a_right`, `s1a_left`, `s1l_right`,
`s1l_left`). The mirrored left arm uses the `s1_left.urdf` axes; if a left joint moves against
its URDF axis, put it in `inverted_joints` for that arm and recalibrate. For a single dataset with
both arms, wrap the two pairs in a small `Robot`/`Teleoperator` subclass that prefixes keys with
`left_`/`right_` (see LeRobot's `bi_so_follower` for the pattern).

## Python API

```python
from lerobot_robot_s1 import S1Follower, S1FollowerConfig, S1Leader, S1LeaderConfig

follower = S1Follower(S1FollowerConfig(id="s1a_right", can_channel="can0"))
leader = S1Leader(S1LeaderConfig(id="s1l_right", port="/dev/ttyUSB0"))
follower.connect(); leader.connect()
obs = follower.get_observation()        # {"base.pos": rad, ..., "gripper.pos": m, "wrist": HxWx3 uint8}
follower.send_action(leader.get_action())
leader.disconnect(); follower.disconnect()
```

`examples/teleop_minimal.py` is the full loop with argument parsing.

### How the follower drives the motors

`lerobot_robot_s1/motors/damiao.py` is a ~400-line clean-room Damiao MIT-mode driver on
python-can. Every cycle each joint gets one MIT frame `(p_des, kp, kd)`; the motor answers with a
status frame that becomes the observation. Torque enable = a few zero-gain frames, then a hold at
the present position, so nothing jumps on power-up. Positions are read only through MIT replies
(never `0xCC` refreshes) so calibration and control share one encoder space.

## Development

```bash
pip install -e ".[dev]"
pytest            # 37 tests, no hardware: fake CAN bus, fake Dynamixel bus, fake camera
ruff check . && ruff format --check .
```

## License

Apache-2.0. See `LICENSE`.
