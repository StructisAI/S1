# Copyright 2026 Structis. Licensed under the Apache License, Version 2.0.
"""Minimal Damiao J-series CAN driver (MIT mode) for the Structis S1A follower.

Clean-room implementation of the parts of the Damiao CAN protocol the S1 kit needs, on top of
`python-can <https://python-can.readthedocs.io>`_ so it runs on Linux (SocketCAN), macOS and
Windows (gs_usb / slcan / pcan adapters).

Protocol summary (all frames are 8 bytes, standard 11-bit ids):

* ``0xFC`` / ``0xFD`` / ``0xFE`` in byte 7 (bytes 0-6 = ``0xFF``) sent to the motor id: enable,
  disable, set-current-position-as-zero.
* MIT control frame sent to the motor id: p_des (16 bit), v_des (12), kp (12), kd (12), t_ff (12),
  each fixed-point over the motor's ``Limit_Param`` (P/V/T max). Motor torque =
  ``kp*(p_des-p) + kd*(v_des-v) + t_ff``. A frame with kp = kd = t_ff = 0 produces no torque and
  is used as a read probe.
* Every control frame is answered by a status frame on the motor's *master id* (``0x10 + id``
  on the S1A): status nibble, position (16), velocity (12), torque (12), MOS temperature.
* ``0x7FF`` broadcast frames: ``[id_lo, id_hi, cmd, rid, value...]`` with cmd ``0xCC`` refresh,
  ``0x33`` read register, ``0x55`` write register, ``0xAA`` save to flash.

Positions are always taken from MIT status replies, never from ``0xCC`` refreshes, so the driver
lives in a single encoder space (dual-encoder -2EC motors can report a small offset between the
two paths).
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass, field

import can

logger = logging.getLogger(__name__)

# (p_max [rad], v_max [rad/s], t_max [Nm]) fixed-point encoding ranges = the firmware Limit_Param
# defaults per model. The real values are read from the motor at connect (RID 21..23) when possible.
MODEL_LIMITS: dict[str, tuple[float, float, float]] = {
    "J8009P": (12.5, 45.0, 54.0),
    "J4340P": (12.5, 10.0, 28.0),
    "J4310": (12.5, 30.0, 10.0),
}

CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD
CMD_SET_ZERO = 0xFE
CMD_REFRESH = 0xCC
CMD_READ_PARAM = 0x33
CMD_WRITE_PARAM = 0x55
CMD_SAVE_PARAM = 0xAA
BROADCAST_ID = 0x7FF

RID_PMAX, RID_VMAX, RID_TMAX = 21, 22, 23
# Registers encoded as uint32 instead of float32.
_INT_RIDS = set(range(7, 11)) | set(range(13, 17)) | {35, 36}

STATUS_DISABLED = 0x0
STATUS_ENABLED = 0x1
STATUS_ERROR_MIN = 0x8  # 0x8 over-voltage, 0x9 under-voltage, 0xA over-current, 0xB MOS temp,
# 0xC coil temp, 0xD comm loss, 0xE overload

REPLY_TIMEOUT_S = 0.02
PING_TIMEOUT_S = 0.5


@dataclass
class DamiaoMotor:
    """One motor on the bus."""

    can_id: int
    model: str
    master_id: int | None = None  # id the motor answers on; S1A uses can_id + 0x10
    kp: float = 0.0
    kd: float = 0.0
    torque_limit: float | None = None  # Nm, measured torque above which the joint yields

    def __post_init__(self) -> None:
        if self.model not in MODEL_LIMITS:
            raise ValueError(f"Unknown Damiao model {self.model!r}, expected one of {list(MODEL_LIMITS)}")
        if self.master_id is None:
            self.master_id = self.can_id + 0x10


@dataclass
class MotorState:
    """Last status frame decoded for a motor."""

    position: float = 0.0  # rad
    velocity: float = 0.0  # rad/s
    torque: float = 0.0  # Nm
    temperature: int = 0  # MOS °C
    status: int = STATUS_DISABLED
    seq: int = 0  # increments on every status frame
    stamp: float = 0.0  # time.monotonic() of the last frame
    params: dict[int, float] = field(default_factory=dict)

    @property
    def error(self) -> bool:
        return self.status >= STATUS_ERROR_MIN


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    return x / ((1 << bits) - 1) * (x_max - x_min) + x_min


def encode_mit(
    p_des: float, v_des: float, kp: float, kd: float, t_ff: float, limits: tuple[float, float, float]
) -> bytes:
    p_max, v_max, t_max = limits
    p = float_to_uint(p_des, -p_max, p_max, 16)
    v = float_to_uint(v_des, -v_max, v_max, 12)
    kp_i = float_to_uint(kp, 0.0, 500.0, 12)
    kd_i = float_to_uint(kd, 0.0, 5.0, 12)
    t = float_to_uint(t_ff, -t_max, t_max, 12)
    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
            kp_i & 0xFF,
            (kd_i >> 4) & 0xFF,
            ((kd_i & 0x0F) << 4) | ((t >> 8) & 0x0F),
            t & 0xFF,
        ]
    )


def decode_status(data: bytes, limits: tuple[float, float, float]) -> tuple[int, float, float, float, int]:
    """Return (status_nibble, position, velocity, torque, mos_temperature)."""
    p_max, v_max, t_max = limits
    status = (data[0] >> 4) & 0x0F
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0x0F) << 8) | data[5]
    return (
        status,
        uint_to_float(p, -p_max, p_max, 16),
        uint_to_float(v, -v_max, v_max, 12),
        uint_to_float(t, -t_max, t_max, 12),
        data[6],
    )


class DamiaoBus:
    """A set of Damiao motors on one CAN bus, driven in MIT mode.

    Lifecycle: ``connect()`` (motors stay torque-free) -> ``enable_torque()`` (motors hold their
    current position) -> ``write_positions()`` / ``read_positions()`` -> ``disconnect()``.
    """

    def __init__(
        self,
        motors: dict[str, DamiaoMotor],
        interface: str = "socketcan",
        channel: str = "can0",
        bitrate: int = 1_000_000,
    ) -> None:
        self.motors = motors
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.limits: dict[str, tuple[float, float, float]] = {
            name: MODEL_LIMITS[m.model] for name, m in motors.items()
        }
        self.states: dict[str, MotorState] = {name: MotorState() for name in motors}
        self.last_goal: dict[str, float] = {}
        self._holding: set[str] = set()
        self._by_master = {m.master_id: name for name, m in motors.items()}
        self._by_can_id = {m.can_id: name for name, m in motors.items()}
        self._bus: can.BusABC | None = None
        self._rx_thread: threading.Thread | None = None
        self._running = False
        self._cond = threading.Condition()
        self._yield_log_stamp: dict[str, float] = {}

    # ------------------------------------------------------------------ lifecycle
    @property
    def is_connected(self) -> bool:
        return self._bus is not None

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError("DamiaoBus already connected")
        self._bus = can.Bus(interface=self.interface, channel=self.channel, bitrate=self.bitrate)
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, name="damiao-rx", daemon=True)
        self._rx_thread.start()
        try:
            self._ping_all()
            self._read_limits()
        except Exception:
            self._close_bus()
            raise
        logger.info(
            "DamiaoBus connected on %s:%s (%d motors)", self.interface, self.channel, len(self.motors)
        )

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self.is_connected:
            return
        try:
            if disable_torque:
                self.disable_torque()
                self.disable_torque()  # repeat, a lost frame here leaves a joint powered
        finally:
            self._close_bus()
        logger.info("DamiaoBus disconnected")

    def _close_bus(self) -> None:
        self._running = False
        bus, self._bus = self._bus, None
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if bus is not None:
            bus.shutdown()
        self._holding.clear()

    def _ping_all(self) -> None:
        seqs = {name: st.seq for name, st in self.states.items()}
        for name in self.motors:
            self._send_refresh(name)
        missing = self._wait_replies(seqs, PING_TIMEOUT_S)
        if missing:
            ids = ", ".join(f"{n} (id {self.motors[n].can_id})" for n in missing)
            raise ConnectionError(
                f"No reply from Damiao motors: {ids}. Check power, CAN wiring/termination and the "
                f"adapter channel ({self.interface}:{self.channel} @ {self.bitrate} bit/s)."
            )

    def _read_limits(self) -> None:
        """Read the encoding limits from each motor so decode/encode match its firmware."""
        for name, motor in self.motors.items():
            vals = [self.read_param(name, rid) for rid in (RID_PMAX, RID_VMAX, RID_TMAX)]
            if any(v is None or not (0.0 < v <= 1000.0) for v in vals):
                logger.debug("%s: could not read Limit_Param, keeping %s defaults", name, motor.model)
                continue
            self.limits[name] = (float(vals[0]), float(vals[1]), float(vals[2]))
            logger.debug("%s: Limit_Param p=%.1f v=%.1f t=%.1f", name, *self.limits[name])

    # ------------------------------------------------------------------ rx side
    def _rx_loop(self) -> None:
        while self._running:
            bus = self._bus
            if bus is None:
                return
            try:
                msg = bus.recv(timeout=0.1)
            except Exception as exc:  # pragma: no cover - adapter specific
                if self._running:
                    logger.debug("CAN recv error: %s", exc)
                continue
            if msg is not None and len(msg.data) == 8:
                self._handle_frame(msg.arbitration_id, bytes(msg.data))

    def _handle_frame(self, arb_id: int, data: bytes) -> None:
        # Register read/write replies: [id_lo, id_hi, 0x33|0x55, rid, value(4)].
        if data[2] in (CMD_READ_PARAM, CMD_WRITE_PARAM) and data[1] == 0 and data[0] in self._by_can_id:
            name = self._by_can_id[data[0]]
            rid = data[3]
            fmt = "<I" if rid in _INT_RIDS else "<f"
            value = struct.unpack(fmt, data[4:8])[0]
            with self._cond:
                self.states[name].params[rid] = value
                self._cond.notify_all()
            return
        name = self._by_master.get(arb_id)
        if name is None:
            # Motor answering on a master id we did not expect: fall back to the id nibble.
            name = self._by_can_id.get(data[0] & 0x0F)
        if name is None:
            return
        status, pos, vel, tau, temp = decode_status(data, self.limits[name])
        with self._cond:
            st = self.states[name]
            st.status, st.position, st.velocity, st.torque, st.temperature = status, pos, vel, tau, temp
            st.seq += 1
            st.stamp = time.monotonic()
            self._cond.notify_all()

    def _wait_replies(self, seqs_before: dict[str, int], timeout: float) -> list[str]:
        """Block until every motor in ``seqs_before`` sent a new status frame. Returns the stragglers."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                missing = [n for n, s in seqs_before.items() if self.states[n].seq == s]
                if not missing:
                    return []
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return missing
                self._cond.wait(remaining)

    # ------------------------------------------------------------------ tx side
    def _send(self, arb_id: int, data: bytes) -> None:
        if self._bus is None:
            raise RuntimeError("DamiaoBus not connected")
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        for attempt in range(3):
            try:
                self._bus.send(msg, timeout=0.01)
                return
            except can.CanError as exc:
                if attempt == 2:
                    raise RuntimeError(f"CAN send failed (id 0x{arb_id:03X}): {exc}") from exc
                time.sleep(0.001)

    def _send_simple(self, name: str, cmd: int) -> None:
        self._send(self.motors[name].can_id, bytes([0xFF] * 7 + [cmd]))

    def _send_refresh(self, name: str) -> None:
        cid = self.motors[name].can_id
        self._send(BROADCAST_ID, bytes([cid & 0xFF, (cid >> 8) & 0xFF, CMD_REFRESH, 0, 0, 0, 0, 0]))

    def _send_mit(
        self, name: str, p_des: float, kp: float, kd: float, v_des: float = 0.0, t_ff: float = 0.0
    ) -> None:
        self._send(self.motors[name].can_id, encode_mit(p_des, v_des, kp, kd, t_ff, self.limits[name]))

    def read_param(self, name: str, rid: int, timeout: float = 0.1) -> float | None:
        cid = self.motors[name].can_id
        with self._cond:
            self.states[name].params.pop(rid, None)
        self._send(BROADCAST_ID, bytes([cid & 0xFF, (cid >> 8) & 0xFF, CMD_READ_PARAM, rid, 0, 0, 0, 0]))
        deadline = time.monotonic() + timeout
        with self._cond:
            while rid not in self.states[name].params:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)
            return self.states[name].params[rid]

    def write_param(self, name: str, rid: int, value: float) -> None:
        cid = self.motors[name].can_id
        payload = struct.pack("<I", int(value)) if rid in _INT_RIDS else struct.pack("<f", float(value))
        self._send(BROADCAST_ID, bytes([cid & 0xFF, (cid >> 8) & 0xFF, CMD_WRITE_PARAM, rid]) + payload)
        time.sleep(0.02)

    # ------------------------------------------------------------------ torque
    def _names(self, motors: str | list[str] | None) -> list[str]:
        if motors is None:
            return list(self.motors)
        return [motors] if isinstance(motors, str) else list(motors)

    def enable_torque(self, motors: str | list[str] | None = None) -> None:
        """Enable and hold each motor at its present position (limp frames first, then gains).

        Fresh-enabled Damiao motors can act on stale MIT parameters left in RAM; a few zero-gain
        frames overwrite those before the position hold is latched, so nothing jumps.
        """
        for name in self._names(motors):
            m = self.motors[name]
            self._send_simple(name, CMD_ENABLE)
            time.sleep(0.002)
            for _ in range(5):
                seq = {name: self.states[name].seq}
                self._send_mit(name, self.states[name].position, 0.0, 0.0)
                self._wait_replies(seq, REPLY_TIMEOUT_S)
            present = self.states[name].position
            self._send_mit(name, present, m.kp, m.kd)
            self.last_goal[name] = present
            self._holding.add(name)
        logger.debug("Torque enabled on %s", self._names(motors))

    def enable_limp(self, motors: str | list[str] | None = None) -> None:
        """Enable motors with zero gains: they answer MIT frames but produce no torque.

        Used while calibrating, so positions come from the same encoder path as during control
        and the arm can still be moved by hand.
        """
        for name in self._names(motors):
            self._holding.discard(name)
            self._send_simple(name, CMD_ENABLE)
            time.sleep(0.002)
            self._send_mit(name, self.states[name].position, 0.0, 0.0)

    def disable_torque(self, motors: str | list[str] | None = None) -> None:
        for name in self._names(motors):
            self._send_simple(name, CMD_DISABLE)
            self._holding.discard(name)
            time.sleep(0.001)

    def is_holding(self, name: str) -> bool:
        return name in self._holding

    def set_zero_position(self, motors: str | list[str] | None = None) -> None:
        """Store the present position as the motor's zero (persistent). Motors must be torque-free."""
        for name in self._names(motors):
            if name in self._holding:
                raise RuntimeError(f"Disable torque on {name!r} before setting its zero position")
            self._send_simple(name, CMD_SET_ZERO)
            time.sleep(0.2)

    # ------------------------------------------------------------------ read / write
    def read_positions(self, motors: str | list[str] | None = None) -> dict[str, float]:
        """Return the present position (rad, motor frame) of each motor.

        Holding motors are re-sent their hold target (which also returns a fresh status frame);
        torque-free motors get a zero-gain probe. Falls back to a ``0xCC`` refresh for motors that
        do not answer MIT frames.
        """
        names = self._names(motors)
        seqs = {n: self.states[n].seq for n in names}
        for n in names:
            if n in self._holding:
                m = self.motors[n]
                self._send_mit(n, self.last_goal[n], m.kp, m.kd)
            else:
                self._send_mit(n, self.states[n].position, 0.0, 0.0)
        missing = self._wait_replies(seqs, REPLY_TIMEOUT_S)
        if missing:
            for n in missing:
                self._send_refresh(n)
            missing = self._wait_replies({n: seqs[n] for n in missing}, REPLY_TIMEOUT_S * 2)
        if missing:
            raise RuntimeError(f"No status reply from motors {missing}; check power and CAN wiring")
        return {n: self.states[n].position for n in names}

    def write_positions(self, goals: dict[str, float]) -> dict[str, float]:
        """Send position targets (rad, motor frame) to holding motors. Returns what was sent.

        A joint whose measured torque exceeds its ``torque_limit`` yields: its target is snapped to
        the present position for this cycle so the PD torque collapses until the load drops.
        """
        sent: dict[str, float] = {}
        seqs: dict[str, int] = {}
        now = time.monotonic()
        for name, goal in goals.items():
            if name not in self._holding:
                raise RuntimeError(f"Motor {name!r} is not torque-enabled")
            m = self.motors[name]
            st = self.states[name]
            target = float(goal)
            if (
                m.torque_limit is not None
                and not st.error
                and now - st.stamp < 0.5
                and abs(st.torque) > m.torque_limit
            ):
                target = st.position
                if now - self._yield_log_stamp.get(name, 0.0) > 1.0:
                    self._yield_log_stamp[name] = now
                    logger.warning(
                        "%s: |torque| %.1f Nm > limit %.1f Nm, yielding", name, st.torque, m.torque_limit
                    )
            seqs[name] = st.seq
            self._send_mit(name, target, m.kp, m.kd)
            self.last_goal[name] = target
            sent[name] = target
        self._wait_replies(seqs, REPLY_TIMEOUT_S)
        return sent
