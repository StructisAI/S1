# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Minimal S1L -> S1A teleoperation using the plugin classes directly.

    python examples/teleop_minimal.py --leader-port /dev/ttyUSB0 --can-channel can0

Both devices must be calibrated first (``lerobot-calibrate``, see README). Ctrl-C to stop; the
follower disables torque on disconnect, so hold the arm or park it on the table before quitting.
"""

from __future__ import annotations

import argparse
import time

from lerobot_robot_s1 import S1Follower, S1FollowerConfig, S1Leader, S1LeaderConfig, default_wrist_camera


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--leader-port", required=True, help="serial port of the S1L (lerobot-find-port)")
    parser.add_argument(
        "--can-interface", default="socketcan", help="python-can interface (socketcan, gs_usb, slcan)"
    )
    parser.add_argument("--can-channel", default="can0", help="python-can channel (can0, 0, /dev/tty...)")
    parser.add_argument("--camera", default=None, help="wrist camera index/path; omit to run without camera")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--follower-id", default="s1a_right")
    parser.add_argument("--leader-id", default="s1l_right")
    args = parser.parse_args()

    cameras = {}
    if args.camera is not None:
        cam = int(args.camera) if args.camera.isdigit() else args.camera
        cameras["wrist"] = default_wrist_camera(cam)

    follower = S1Follower(
        S1FollowerConfig(
            id=args.follower_id,
            can_interface=args.can_interface,
            can_channel=args.can_channel,
            cameras=cameras,
        )
    )
    leader = S1Leader(S1LeaderConfig(id=args.leader_id, port=args.leader_port))

    follower.connect()
    leader.connect()
    period = 1.0 / args.fps
    try:
        while True:
            t0 = time.perf_counter()
            action = leader.get_action()  # {"base.pos": rad, ..., "gripper.pos": m}
            observation = follower.get_observation()  # same keys + camera images
            follower.send_action(action)
            joints = {k.removesuffix(".pos"): v for k, v in observation.items() if k.endswith(".pos")}
            print(" ".join(f"{k}={v:+.2f}" for k, v in joints.items()), end="\r", flush=True)
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        leader.disconnect()
        follower.disconnect()


if __name__ == "__main__":
    main()
