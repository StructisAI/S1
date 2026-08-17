# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Test fixtures: a fake python-can bus that behaves like Damiao motors, and a fake camera."""

from __future__ import annotations

import queue
import struct
from dataclasses import dataclass, field

import can
import numpy as np
import pytest

from lerobot_robot_s1.motors import damiao as dm

STATUS_LEN = 8


@dataclass
class FakeDamiaoMotor:
    can_id: int
    master_id: int
    limits: tuple[float, float, float]
    position: float = 0.0
    torque: float = 0.0
    enabled: bool = False
    error: int = 0  # status nibble override when >= 8
    params: dict[int, float] = field(default_factory=dict)
    zero_calls: int = 0


class FakeCanBus:
    """Emulates a set of Damiao motors behind the ``can.BusABC`` send/recv interface.

    * MIT frames with kp > 0 move the motor to ``p_des`` instantly (perfect tracking).
    * Every control frame is answered with a status frame on the master id.
    * ``reply_to_mit_when_disabled=False`` makes disabled motors ignore MIT frames (only 0xCC
      refreshes are answered) to exercise the driver's fallback path.
    """

    def __init__(self, motors: dict[str, FakeDamiaoMotor], reply_to_mit_when_disabled: bool = True):
        self.motors = motors
        self._by_id = {m.can_id: m for m in motors.values()}
        self.reply_to_mit_when_disabled = reply_to_mit_when_disabled
        self.rx: queue.Queue[can.Message] = queue.Queue()
        self.sent: list[can.Message] = []
        self.shut_down = False

    # -- can.BusABC surface -------------------------------------------------
    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        self.sent.append(msg)
        self._handle(msg.arbitration_id, bytes(msg.data))

    def recv(self, timeout: float | None = None):
        try:
            return self.rx.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self) -> None:
        self.shut_down = True

    # -- helpers for assertions -----------------------------------------------
    def frames_to(self, can_id: int) -> list[bytes]:
        return [bytes(m.data) for m in self.sent if m.arbitration_id == can_id]

    def simple_commands(self, can_id: int) -> list[int]:
        return [d[7] for d in self.frames_to(can_id) if d[:7] == b"\xff" * 7]

    # -- emulation --------------------------------------------------------------
    def _status(self, m: FakeDamiaoMotor) -> None:
        p_max, v_max, t_max = m.limits
        status = m.error if m.error >= 8 else (1 if m.enabled else 0)
        p = dm.float_to_uint(m.position, -p_max, p_max, 16)
        v = dm.float_to_uint(0.0, -v_max, v_max, 12)
        t = dm.float_to_uint(m.torque, -t_max, t_max, 12)
        data = bytes(
            [
                (status << 4) | (m.can_id & 0x0F),
                (p >> 8) & 0xFF,
                p & 0xFF,
                (v >> 4) & 0xFF,
                ((v & 0x0F) << 4) | ((t >> 8) & 0x0F),
                t & 0xFF,
                30,
                0,
            ]
        )
        self.rx.put(can.Message(arbitration_id=m.master_id, data=data, is_extended_id=False))

    def _handle(self, arb_id: int, data: bytes) -> None:
        if arb_id == dm.BROADCAST_ID:
            cid = data[0] | (data[1] << 8)
            m = self._by_id.get(cid)
            if m is None:
                return
            cmd, rid = data[2], data[3]
            if cmd == dm.CMD_REFRESH:
                self._status(m)
            elif cmd == dm.CMD_READ_PARAM:
                value = m.params.get(rid)
                if value is None:
                    return
                payload = struct.pack("<I", int(value)) if rid in dm._INT_RIDS else struct.pack("<f", value)
                self.rx.put(
                    can.Message(
                        arbitration_id=m.master_id,
                        data=bytes([data[0], data[1], dm.CMD_READ_PARAM, rid]) + payload,
                        is_extended_id=False,
                    )
                )
            elif cmd == dm.CMD_WRITE_PARAM:
                fmt = "<I" if rid in dm._INT_RIDS else "<f"
                m.params[rid] = struct.unpack(fmt, data[4:8])[0]
            return
        m = self._by_id.get(arb_id)
        if m is None:
            return
        if data[:7] == b"\xff" * 7:
            cmd = data[7]
            if cmd == dm.CMD_ENABLE:
                m.enabled = True
            elif cmd == dm.CMD_DISABLE:
                m.enabled = False
            elif cmd == dm.CMD_SET_ZERO:
                m.position = 0.0
                m.zero_calls += 1
            return
        # MIT frame
        if not m.enabled and not self.reply_to_mit_when_disabled:
            return
        p_max, _, _ = m.limits
        p_des = dm.uint_to_float((data[0] << 8) | data[1], -p_max, p_max, 16)
        kp = dm.uint_to_float(((data[3] & 0x0F) << 8) | data[4], 0.0, 500.0, 12)
        if m.enabled and kp > 0.0:
            m.position = p_des
        self._status(m)


class FakeCamera:
    def __init__(self, height: int = 480, width: int = 640):
        self.height, self.width = height, width
        self.is_connected = False

    def connect(self, warmup: bool = True) -> None:
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def async_read(self, timeout_ms: float = 200) -> np.ndarray:
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)


def make_fake_motors(models: dict[str, str], ids: dict[str, int], positions: dict[str, float] | None = None):
    positions = positions or {}
    return {
        name: FakeDamiaoMotor(
            can_id=ids[name],
            master_id=ids[name] + 0x10,
            limits=dm.MODEL_LIMITS[model],
            position=positions.get(name, 0.0),
            params={
                dm.RID_PMAX: dm.MODEL_LIMITS[model][0],
                dm.RID_VMAX: dm.MODEL_LIMITS[model][1],
                dm.RID_TMAX: dm.MODEL_LIMITS[model][2],
            },
        )
        for name, model in models.items()
    }


@pytest.fixture
def patch_can(monkeypatch):
    """Patch ``can.Bus`` so DamiaoBus talks to a FakeCanBus. Returns a factory to install one."""
    holder: dict[str, FakeCanBus] = {}

    def install(fake: FakeCanBus) -> FakeCanBus:
        holder["bus"] = fake
        monkeypatch.setattr(dm.can, "Bus", lambda **kwargs: fake)
        return fake

    return install
