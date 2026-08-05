"""Binary protocol shared by the RA8P1 robot firmware and the PC controller."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable

HEADER = b"\xA5\x5A"
VERSION = 3
MSG_CONTROL = 0x01
MSG_CONFIG = 0x02
MSG_TELEMETRY = 0x81
MAX_PAYLOAD = 96

FLAG_ARM = 1 << 0
FLAG_DISARM = 1 << 1
FLAG_HOME = 1 << 2
FLAG_CLEAR_FAULT = 1 << 3
FLAG_CALIBRATE = 1 << 4

STATE_NAMES = {
    0: "未使能/未回零",
    1: "回零中",
    2: "未使能/已回零",
    3: "已使能",
    4: "故障",
    5: "标定软上限",
    6: "自动恢复中",
}

FAULT_NAMES = {
    1 << 0: "LoRa 控制超时",
    1 << 1: "舵机掉线",
    1 << 2: "舵机状态/过温",
    1 << 3: "回零超时",
    1 << 4: "驱动初始化失败",
    1 << 5: "回零堵转/无位移",
}

SERVO_STATUS_NAMES = {
    1 << 0: "电压",
    1 << 1: "传感器",
    1 << 2: "温度",
    1 << 3: "电流",
    1 << 4: "角度",
    1 << 5: "过载",
}


def _servo_fault_text(protocol_error: int, status_flags: int, temperature_c: int) -> str:
    raw = protocol_error | status_flags
    names = [name for bit, name in SERVO_STATUS_NAMES.items() if raw & bit]
    if raw & ~0x3F:
        names.append("未知状态")
    if temperature_c >= 70 and "温度" not in names:
        names.append("温度")
    if not names:
        return "状态正常"
    return f"异常:{'/'.join(names)} E=0x{protocol_error:02X} S=0x{status_flags:02X}"


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload is too large")
    body = struct.pack("<BBHBB", VERSION, message_type, sequence & 0xFFFF, len(payload), 0) + payload
    return HEADER + body + struct.pack("<H", crc16_ccitt(body))


def encode_control(sequence: int, vx: int, vy: int, omega: int, lift: int, flags: int = 0) -> bytes:
    values = [max(-1000, min(1000, int(value))) for value in (vx, vy, omega, lift)]
    return encode_frame(MSG_CONTROL, sequence, struct.pack("<hhhhH", *values, flags & 0xFFFF))


def encode_config(sequence: int, upper_limit_counts: int) -> bytes:
    return encode_frame(MSG_CONFIG, sequence, struct.pack("<i", int(upper_limit_counts)))


@dataclass(frozen=True)
class Frame:
    message_type: int
    sequence: int
    payload: bytes


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes | bytearray | Iterable[int]) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer.endswith(HEADER[:1]) else b""
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 10:
                break
            version, message_type, sequence, length, _reserved = struct.unpack_from("<BBHBB", self._buffer, 2)
            if version != VERSION or length > MAX_PAYLOAD:
                del self._buffer[0]
                continue
            total = 10 + length
            if len(self._buffer) < total:
                break
            body = bytes(self._buffer[2 : 8 + length])
            received_crc = struct.unpack_from("<H", self._buffer, 8 + length)[0]
            if crc16_ccitt(body) == received_crc:
                frames.append(Frame(message_type, sequence, bytes(self._buffer[8 : 8 + length])))
                del self._buffer[:total]
            else:
                del self._buffer[0]
        return frames


@dataclass(frozen=True)
class ServoTelemetry:
    online: bool
    protocol_error: int
    status_flags: int
    temperature_c: int
    temperature_limit_c: int | None
    voltage_v: float
    position: int
    speed: int
    current: int

    @property
    def status_error(self) -> bool:
        return bool(self.protocol_error or self.status_flags)

    @property
    def fault_text(self) -> str:
        return _servo_fault_text(self.protocol_error, self.status_flags, self.temperature_c)


@dataclass(frozen=True)
class FaultSnapshot:
    valid: bool
    servo_id: int
    protocol_error: int
    status_flags: int
    temperature_c: int
    temperature_limit_c: int | None

    @property
    def fault_text(self) -> str:
        if not self.valid:
            return "无"
        limit = f"{self.temperature_limit_c}°C" if self.temperature_limit_c is not None else "--"
        return (
            f"ID{self.servo_id} {_servo_fault_text(self.protocol_error, self.status_flags, self.temperature_c)}，"
            f"触发温度 {self.temperature_c}°C / 舵机上限 {limit}"
        )


@dataclass(frozen=True)
class Telemetry:
    state: int
    fault_bits: int
    last_control_sequence: int
    link_age_ms: int
    online_mask: int
    warning_mask: int
    home_switch: bool
    homed: bool
    lift_position: int
    lift_target: int
    upper_limit: int
    wheel_speed: tuple[int, int, int]
    servos: tuple[ServoTelemetry, ...]
    fault_snapshot: FaultSnapshot

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f"未知({self.state})")

    @property
    def fault_text(self) -> str:
        names = []
        for bit, name in FAULT_NAMES.items():
            if not self.fault_bits & bit:
                continue
            if bit == (1 << 2):
                if self.fault_snapshot.valid:
                    names.append(self.fault_snapshot.fault_text)
                    continue
                details = [f"ID{index + 1} {servo.fault_text}" for index, servo in enumerate(self.servos) if servo.status_error]
                names.append("；".join(details) if details else f"{name}（状态已恢复或温度阈值触发）")
            else:
                names.append(name)
        return "、".join(names) if names else "无"


def decode_telemetry(payload: bytes) -> Telemetry:
    if len(payload) != 85:
        raise ValueError(f"unexpected telemetry length: {len(payload)}")
    state = payload[0]
    fault_bits = struct.unpack_from("<I", payload, 1)[0]
    last_sequence, link_age = struct.unpack_from("<HH", payload, 5)
    online_mask, warning_mask, home_switch, homed = payload[9:13]
    lift_position, lift_target, upper_limit = struct.unpack_from("<iii", payload, 13)
    wheel_speed = struct.unpack_from("<hhh", payload, 25)
    servos = []
    for index in range(4):
        offset = 31 + index * 12
        online, protocol_error, status_flags, temperature, temperature_limit, voltage = struct.unpack_from(
            "<BBBBBB", payload, offset
        )
        position, speed, current = struct.unpack_from("<hhh", payload, offset + 6)
        servos.append(
            ServoTelemetry(
                online=bool(online),
                protocol_error=protocol_error,
                status_flags=status_flags,
                temperature_c=temperature,
                temperature_limit_c=temperature_limit or None,
                voltage_v=voltage / 10.0,
                position=position,
                speed=speed,
                current=current,
            )
        )
    snapshot_valid, snapshot_id, snapshot_error, snapshot_status, snapshot_temp, snapshot_limit = struct.unpack_from(
        "<BBBBBB", payload, 79
    )
    snapshot = FaultSnapshot(
        valid=bool(snapshot_valid),
        servo_id=snapshot_id,
        protocol_error=snapshot_error,
        status_flags=snapshot_status,
        temperature_c=snapshot_temp,
        temperature_limit_c=snapshot_limit or None,
    )
    return Telemetry(
        state,
        fault_bits,
        last_sequence,
        link_age,
        online_mask,
        warning_mask,
        bool(home_switch),
        bool(homed),
        lift_position,
        lift_target,
        upper_limit,
        wheel_speed,
        tuple(servos),
        snapshot,
    )
