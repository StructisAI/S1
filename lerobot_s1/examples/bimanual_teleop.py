# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Two S1L/S1A pairs (left + right) teleoperated from one loop.

    python examples/bimanual_teleop.py \
        --right-leader-port /dev/ttyUSB0 --right-can can0 \
        --left-leader-port  /dev/ttyUSB1 --left-can  can1

Each arm is its own LeRobot device with its own id (calibration file) and CAN channel; only the
left follower gets ``inverted_joints`` if your mirrored build needs it (see README, Bimanual).
"""

from __future__ import annotations

import argparse
import time

from lerobot_robot_s1 import S1Follower, S1FollowerConfig, S1Leader, S1LeaderConfig, default_wrist_camera


def make_pair(side: str, leader_port: str, can_channel: str, can_interface: str, camera):
    cameras = {} if camera is None else {"wrist": default_wrist_camera(camera)}
    cfg = S1FollowerConfig(
        id=f"s1a_{side}", can_interface=can_interface, can_channel=can_channel, cameras=cameras
    )
    follower = S1Follower(cfg)
    leader = S1Leader(S1LeaderConfig(id=f"s1l_{side}", port=leader_port))
    return follower, leader


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    for side in ("right", "left"):
        parser.add_argument(f"--{side}-leader-port", required=True)
        parser.add_argument(f"--{side}-can", required=True, help="CAN channel of this arm's adapter")
        parser.add_argument(f"--{side}-camera", type=int, default=None)
    parser.add_argument("--can-interface", default="socketcan")
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    pairs = {
        side: make_pair(
            side,
            getattr(args, f"{side}_leader_port"),
            getattr(args, f"{side}_can"),
            args.can_interface,
            getattr(args, f"{side}_camera"),
        )
        for side in ("right", "left")
    }
    for follower, leader in pairs.values():
        follower.connect()
        leader.connect()
    period = 1.0 / args.fps
    try:
        while True:
            t0 = time.perf_counter()
            for follower, leader in pairs.values():
                # For a bimanual dataset, prefix keys per side (left_/right_) before storing.
                follower.send_action(leader.get_action())
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        for follower, leader in pairs.values():
            leader.disconnect()
            follower.disconnect()


if __name__ == "__main__":
    main()
