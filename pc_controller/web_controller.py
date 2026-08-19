"""Web (browser) controller for the RA8P1 omni robot.

Serves an Apple-style single-page UI and bridges it to the LoRa serial port.
All wire-protocol and safety logic stays in this backend (single sender,
200 ms half-duplex slots, sequence sync, safe-stop); the browser only sends
high-level intent over a WebSocket and renders telemetry.

Run:  py web_controller.py  then open http://127.0.0.1:8080/
"""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
import queue
import re
import shutil
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Reported to the browser on connect attempt.
    serial = None
    list_ports = None

from aiohttp import web

from protocol import (
    FAULT_NAMES,
    FLAG_ARM,
    FLAG_CLEAR_FAULT,
    FLAG_DISARM,
    MSG_TELEMETRY,
    FrameParser,
    Telemetry,
    decode_telemetry,
    encode_control,
    encode_face_event,
)

BASE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = BASE_DIR / "controller_config.json"
INDEX_PATH = BASE_DIR / "web" / "index.html"

DEFAULT_CONFIG = {
    "com_port": "",
    "baud": 115200,
    "speed_scale": 0.20,
    "lift_speed_scale": 0.60,
    "gamepad_deadzone": 0.12,
    "gamepad_axes": {"left_x": 0, "left_y": 1, "right_x": 2},
    "gamepad_buttons": {
        "left_shoulder": 4,
        "right_shoulder": 5,
        "start": 7,
        "stop": 1,
        "face_startup": 2,
        "face_charge_start": 3,
        "face_charge_complete": 0,
        "face_safe_dock": 1,
    },
    # 一键任务（自动充电演示）：底盘定时脉冲 + 语音事件 + 机械臂预制动作。
    # 各时长单位秒；sim_* 只影响 3D 演示动画，实物速度仍由 speed_scale 决定。
    "task": {
        "forward_to_pile_s": 3.5,      # 原地出发，前进到充电桩
        "turn_s": 4.5,                 # 原地右转 ~90°（已按实车标定）
        "forward_to_car_s": 11.0,      # 右转后前进到汽车旁
        "arm_action_code": "TASK-20260420-001",  # ACT Store 预制动作（向左边张嘴），取枪/插枪共用
        # 一键任务中执行动作的机械臂（单臂：左臂）
        "arm_device": "B2R-2805A54FE4B4",
        # 聊天快捷动作执行的机械臂（左右两臂同时执行同一动作，保持同步）
        "arm_devices": ["B2R-2805A54FE4B4", "B2R-2805A54E0140"],
        "arm_action_s": 10.0,          # 等待机械臂动作完成的时长
        "lift_up_s": 2.5,              # 到桩后、取枪前，升降台升高时长
        "sim_move": 0.22,              # 3D 演示平移速度（满幅的比例，按行程适配场地）
        "sim_turn": 0.205,             # 3D 演示转向速度（约 90°/turn_s）
        # 每条语音的完整播放时长：Linux 端收到新事件会重启语音，
        # 因此下一条语音必须等上一条播完这么久之后再发。
        "voice_play_s": {
            "1": 3.0,   # 启动
            "3": 2.0,   # 机器人移动中
            "5": 2.0,   # 请停止
            "6": 2.5,   # 开始充电
            "9": 2.5,   # 任务完成
            "10": 2.0,  # 安全停靠
        },
    },
    # 聊天助手快捷动作：消息与键名完全一致时跳过 Kimi 大模型，
    # 直接在 task.arm_devices 指定的机械臂（默认左右双臂同步）上执行对应 ACT Store 动作。
    "quick_actions": {
        "伸懒腰": "TASK-20260420-008",
        "张嘴": "TASK-20260420-007",
        "向左边张嘴": "TASK-20260420-001",
        "向右边张嘴": "TASK-20260420-002",
        "点头": "TASK-20260420-006",
        "点头yes": "TASK-20260420-006",
        "挥手": "TASK-20260420-005",
        "挥挥手": "TASK-20260420-005",
        "朝左边点头": "TASK-20260420-004",
        "向右边点头": "TASK-20260420-003",
        "比爱心": "TASK-20260602-001",
    },
}

# ATK-MWCC68D is half duplex. Two control slots are aligned from the
# first telemetry frame, then the rest of each 200 ms period is silent.
CONTROL_SUPERFRAME_S = 0.200
CONTROL_TX_OFFSETS_S = (0.020, 0.070)
TX_TICK_S = 0.005
CLIENT_STALE_S = 1.0
HOST = "127.0.0.1"
PORT = 8080

JLINK_VID = 0x1366

# Chat assistant: forwards browser chat messages to a Kimi Code CLI session
# (which owns the box2robot skill) and relays the reply back.
KIMI_EXE = shutil.which("kimi") or str(Path.home() / ".kimi-code" / "bin" / "kimi.exe")
CHAT_WORKDIR = Path.home() / ".kimi-code" / "b2r-chat"
CHAT_TIMEOUT_S = 180


class ChatBridge:
    """One Kimi Code CLI session shared by all browser clients."""

    def __init__(self) -> None:
        self.session_id: str | None = None
        self.busy = False

    @staticmethod
    def _tool_summary(event: dict) -> str:
        """Short human-readable summary of a tool call event."""
        for call in event.get("tool_calls") or []:
            fn = call.get("function") or {}
            name = fn.get("name", "tool")
            try:
                args = json.loads(fn.get("arguments") or "{}")
            except ValueError:
                args = {}
            detail = args.get("command") or args.get("path") or args.get("prompt") or ""
            detail = " ".join(str(detail).split())[:120]
            return f"{name}: {detail}" if detail else name
        return "tool"

    async def ask_stream(self, text: str, on_event) -> None:
        """Run one prompt, forwarding assistant segments/tool calls as they happen."""
        CHAT_WORKDIR.mkdir(parents=True, exist_ok=True)
        cmd = [KIMI_EXE]
        if self.session_id:
            cmd += ["-r", self.session_id]
        cmd += ["-p", text, "--output-format", "stream-json"]
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            cwd=str(CHAT_WORKDIR),
            env=env,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )

        async def pump() -> None:
            assert proc.stdout is not None
            while True:
                line = await proc.stdout.readline()
                if not line:
                    break
                try:
                    event = json.loads(line.decode("utf-8", errors="replace"))
                except ValueError:
                    continue  # non-JSON chatter (raw tool stdout etc.)
                role = event.get("role")
                if role == "assistant":
                    if event.get("content"):
                        await on_event({"type": "chat_delta", "text": event["content"]})
                    if event.get("tool_calls"):
                        await on_event({"type": "chat_tool", "text": self._tool_summary(event)})
                elif role == "meta" and event.get("type") == "session.resume_hint":
                    if event.get("session_id"):
                        self.session_id = event["session_id"]
            await proc.wait()

        try:
            await asyncio.wait_for(pump(), timeout=CHAT_TIMEOUT_S)
        except asyncio.TimeoutError:
            proc.kill()
            await on_event({
                "type": "chat_delta",
                "text": f"（Kimi Code 超过 {CHAT_TIMEOUT_S} 秒没有响应，已终止；可以点“新会话”重试）",
            })


def load_config() -> dict:
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    if CONFIG_PATH.exists():
        try:
            saved = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            for key, value in saved.items():
                if key not in config:
                    continue
                if isinstance(config[key], dict) and isinstance(value, dict):
                    config[key].update(value)
                else:
                    config[key] = value
        except (OSError, ValueError):
            pass
    return config


def save_config(config: dict) -> None:
    CONFIG_PATH.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")


def telemetry_to_dict(telemetry: Telemetry) -> dict:
    return {
        "state": telemetry.state,
        "state_name": telemetry.state_name,
        "fault_bits": telemetry.fault_bits,
        "fault_text": telemetry.fault_text,
        "last_control_sequence": telemetry.last_control_sequence,
        "link_age_ms": telemetry.link_age_ms,
        "online_mask": telemetry.online_mask,
        "warning_mask": telemetry.warning_mask,
        "home_switch": telemetry.home_switch,
        "homed": telemetry.homed,
        "lift_position": telemetry.lift_position,
        "lift_target": telemetry.lift_target,
        "upper_limit": telemetry.upper_limit,
        "wheel_speed": list(telemetry.wheel_speed),
        "servos": [
            {
                "online": servo.online,
                "protocol_error": servo.protocol_error,
                "status_flags": servo.status_flags,
                "temperature_c": servo.temperature_c,
                "temperature_limit_c": servo.temperature_limit_c,
                "voltage_v": round(servo.voltage_v, 1),
                "position": servo.position,
                "speed": servo.speed,
                "current": servo.current,
                "fault_text": servo.fault_text,
            }
            for servo in telemetry.servos
        ],
        "fault_snapshot": {
            "valid": telemetry.fault_snapshot.valid,
            "servo_id": telemetry.fault_snapshot.servo_id,
            "protocol_error": telemetry.fault_snapshot.protocol_error,
            "status_flags": telemetry.fault_snapshot.status_flags,
            "temperature_c": telemetry.fault_snapshot.temperature_c,
            "temperature_limit_c": telemetry.fault_snapshot.temperature_limit_c,
            "fault_text": telemetry.fault_snapshot.fault_text,
        },
    }


class RobotBackend:
    """Owns the serial port and is the only place that assigns frame sequences."""

    def __init__(self) -> None:
        self.config = load_config()
        self.port = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.incoming: asyncio.Queue = asyncio.Queue()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.clients: set[web.WebSocketResponse] = set()

        self.connected_at = 0.0
        self.sequence = 0
        self.sequence_synced = False
        self.pending_flags = FLAG_DISARM
        self.pending_face_event: int | None = None
        self.telemetry: Telemetry | None = None
        self.last_telemetry_at = 0.0
        self.tx_epoch = time.monotonic()
        self.tx_cycle = 0
        self.tx_next_slot = 0

        self.motion = (0.0, 0.0, 0.0, 0.0)  # raw -1..1 intent from the browser
        self.chat_motion = None  # (values, expires_at) timed override from the chat assistant
        self.last_client_msg_at = 0.0
        self.serial_error: str | None = None
        self.write_failures = 0
        self.chat = ChatBridge()
        self.task_run: asyncio.Task | None = None  # one-click task choreography
        self._task_last_voice: tuple[int, float] | None = None  # (code, 发出的 monotonic 时刻)

    # ------------------------------------------------------------------ serial

    def list_ports(self) -> list[dict]:
        if list_ports is None:
            return []
        ports = []
        for item in list_ports.comports():
            ports.append(
                {
                    "device": item.device,
                    "description": item.description or "",
                    "is_jlink": item.vid == JLINK_VID,
                }
            )
        return ports

    def connect(self, port_name: str) -> str | None:
        """Returns an error string, or None on success."""
        if serial is None:
            return "缺少依赖：请先运行 pip install -r requirements.txt"
        if self.port is not None:
            return "串口已连接，请先断开"
        port_name = (port_name or "").strip()
        if not port_name:
            return "未选择串口"
        if list_ports is not None:
            info = next((p for p in list_ports.comports() if p.device == port_name), None)
            if info is not None and info.vid == JLINK_VID:
                return f"{port_name} 是 J-Link 调试器，不是电脑端 LoRa 串口"
        try:
            self.port = serial.Serial(port_name, int(self.config["baud"]), timeout=0.05, write_timeout=0.5)
        except Exception as exc:
            self.port = None
            return str(exc)
        self.reader_stop.clear()
        self.connected_at = time.monotonic()
        self.telemetry = None
        self.sequence_synced = False
        self.pending_flags |= FLAG_DISARM
        self.pending_face_event = None
        self.motion = (0.0, 0.0, 0.0, 0.0)
        self.serial_error = None
        self.write_failures = 0
        self.tx_epoch = time.monotonic()
        self.tx_cycle = 0
        self.tx_next_slot = 0
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        print(f"[serial] 已打开 {port_name} @ {int(self.config['baud'])}", flush=True)
        self.config["com_port"] = port_name
        save_config(self.config)
        return None

    async def disconnect(self, safe_stop: bool = True) -> None:
        if self.task_running():
            self.task_run.cancel()
            self.task_run = None
        if safe_stop and self.port is not None:
            await self._send_safe_stop(3)
        self.reader_stop.set()
        if self.reader_thread is not None:
            await asyncio.to_thread(self.reader_thread.join, 0.3)
            self.reader_thread = None
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.sequence_synced = False
        self.telemetry = None
        self.pending_face_event = None
        self.motion = (0.0, 0.0, 0.0, 0.0)
        self.chat_motion = None

    def _reader_loop(self) -> None:
        parser = FrameParser()
        while not self.reader_stop.is_set() and self.port is not None:
            try:
                data = self.port.read(256)
                for frame in parser.feed(data):
                    if frame.message_type == MSG_TELEMETRY:
                        self._threadsafe_put(decode_telemetry(frame.payload))
            except Exception as exc:
                print(f"[serial] 读取错误: {exc}", flush=True)
                self._threadsafe_put(exc)
                break

    def _threadsafe_put(self, item) -> None:
        if self.loop is not None and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.incoming.put_nowait, item)

    # --------------------------------------------------------------- transmit

    def _next_sequence(self) -> int:
        value = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        return value

    def _write(self, data: bytes) -> bool:
        if self.port is None or not self.port.is_open:
            return False
        try:
            self.port.write(data)
            self.write_failures = 0
            return True
        except Exception as exc:
            # CH340/USB hubs occasionally stall a single write; only treat
            # repeated failures as a dead port (read errors still disconnect
            # immediately in _reader_loop, e.g. when the cable is unplugged).
            self.write_failures += 1
            print(f"[serial] write 失败第 {self.write_failures} 次: {exc}", flush=True)
            if self.write_failures >= 5:
                self._threadsafe_put(exc)
            return False

    async def _send_safe_stop(self, count: int) -> None:
        self.motion = (0.0, 0.0, 0.0, 0.0)
        for _ in range(count):
            self._write(encode_control(self._next_sequence(), 0, 0, 0, 0, FLAG_DISARM))
            await asyncio.sleep(0.05)

    def stop_now(self) -> None:
        """SPACE semantics: bypass the TX slots, then let slots keep repeating."""
        self.motion = (0.0, 0.0, 0.0, 0.0)
        self.pending_flags |= FLAG_DISARM
        self._write(encode_control(self._next_sequence(), 0, 0, 0, 0, FLAG_DISARM))

    def _control_slot_due(self, now: float) -> int | None:
        elapsed = max(0.0, now - self.tx_epoch)
        cycle = int(elapsed / CONTROL_SUPERFRAME_S)
        if cycle != self.tx_cycle:
            self.tx_cycle = cycle
            self.tx_next_slot = 0
        cycle_elapsed = elapsed - cycle * CONTROL_SUPERFRAME_S
        if self.tx_next_slot >= len(CONTROL_TX_OFFSETS_S):
            return None
        if cycle_elapsed < CONTROL_TX_OFFSETS_S[self.tx_next_slot]:
            return None
        slot = self.tx_next_slot
        self.tx_next_slot += 1
        return slot

    def _effective_motion(self) -> tuple:
        """Chat-commanded timed motion wins over browser input until it expires."""
        if self.chat_motion is not None:
            values, expires_at = self.chat_motion
            if time.monotonic() < expires_at:
                return values
            self.chat_motion = None
        return self.motion

    def _scaled_motion(self) -> tuple[int, int, int, int]:
        chassis_scale = max(0.05, min(1.0, float(self.config.get("speed_scale", 0.2))))
        lift_scale = max(0.10, min(1.0, float(self.config.get("lift_speed_scale", 0.6))))
        vx, vy, omega, lift = (max(-1.0, min(1.0, value)) for value in self._effective_motion())
        return (
            int(round(vx * chassis_scale * 1000.0)),
            int(round(vy * chassis_scale * 1000.0)),
            int(round(omega * chassis_scale * 1000.0)),
            int(round(lift * lift_scale * 1000.0)),
        )

    # -------------------------------------------------------------- telemetry

    def _handle_telemetry(self, telemetry: Telemetry) -> None:
        if not self.sequence_synced:
            # A restarted PC app begins with a new local sequence counter, while
            # the running MCU still remembers the previous session's counter.
            self.sequence = (telemetry.last_control_sequence + 1) & 0xFFFF
            self.sequence_synced = True
            self.pending_flags |= FLAG_DISARM
            self.tx_epoch = time.monotonic()
            self.tx_cycle = 0
            self.tx_next_slot = 0
        self.telemetry = telemetry
        self.last_telemetry_at = time.monotonic()

    # ------------------------------------------------------------ client input

    def set_motion(self, vx, vy, omega, lift) -> None:
        values = []
        for value in (vx, vy, omega, lift):
            try:
                values.append(max(-1.0, min(1.0, float(value))))
            except (TypeError, ValueError):
                values.append(0.0)
        self.motion = tuple(values)

    def pulse_flag(self, flag: int) -> None:
        self.pending_flags |= flag

    def set_chat_motion(self, vx, vy, omega, lift, duration_ms) -> str | None:
        """Timed motion pulse from the chat assistant. Returns an error string, or None."""
        if self.port is None or not self.sequence_synced:
            return "串口未连接或控制序号尚未同步"
        if self.telemetry is None or time.monotonic() - self.last_telemetry_at > 1.0:
            return "遥测失联，拒绝聊天运动指令"
        try:
            values = tuple(max(-1.0, min(1.0, float(v))) for v in (vx, vy, omega, lift))
            ms = max(100, min(3000, int(duration_ms)))
        except (TypeError, ValueError):
            return "参数无效：分量须在 -1~1，时长须为毫秒整数"
        self.chat_motion = (values, time.monotonic() + ms / 1000.0)
        return None

    async def queue_face_event(self, code) -> None:
        if self.port is None or not self.sequence_synced:
            await self.broadcast({"type": "error", "message": "串口未连接或控制序号尚未同步"})
            return
        try:
            event_code = int(code)
        except (TypeError, ValueError):
            event_code = -1
        if not 0 <= event_code <= 0x0A:
            await self.broadcast({"type": "error", "message": "无效的屏幕/语音事件"})
            return
        self.pending_face_event = event_code
        await self.broadcast({"type": "face_event_status", "state": "queued", "code": event_code})

    def toggle_arm(self) -> None:
        active = self.telemetry is not None and self.telemetry.state in (1, 3, 5, 6)
        self.pulse_flag(FLAG_DISARM if active else FLAG_ARM)

    # ------------------------------------------------------------ one-click task

    def task_running(self) -> bool:
        return self.task_run is not None and not self.task_run.done()

    def _link_live(self) -> bool:
        """真实链路可用（非演示模式）：串口已连接且遥测新鲜。"""
        return (
            self.port is not None
            and self.sequence_synced
            and self.telemetry is not None
            and time.monotonic() - self.last_telemetry_at < 1.0
        )

    async def start_one_click_task(self) -> None:
        if self.task_running():
            await self.broadcast({"type": "task_status", "state": "busy", "label": "任务已在进行中"})
            return
        self.task_run = asyncio.create_task(self._run_task())

    async def cancel_one_click_task(self, reason: str = "手动中断") -> None:
        if not self.task_running():
            return
        self.task_run.cancel()
        try:
            await self.task_run
        except (asyncio.CancelledError, Exception):
            pass
        self.task_run = None

    def _voice_play_s(self, code: int) -> float:
        cfg = dict(DEFAULT_CONFIG["task"], **(self.config.get("task") or {}))
        table = cfg.get("voice_play_s") or {}
        try:
            return float(table.get(str(code), 2.0))
        except (TypeError, ValueError):
            return 2.0

    async def _task_say(self, code: int, label: str, wait_prev: bool = True) -> None:
        """排队一个屏幕/语音事件；演示模式下只在界面上显示，不下发。

        Linux 端收到新事件会重启语音，因此默认先等上一条语音播完再发；
        「请停止」这类中断提示用 wait_prev=False 立即打断。
        """
        if wait_prev and self._task_last_voice is not None:
            prev_code, prev_at = self._task_last_voice
            remaining = self._voice_play_s(prev_code) - (time.monotonic() - prev_at)
            if remaining > 0:
                await asyncio.sleep(remaining)
        live = self.port is not None and self.sequence_synced
        if live:
            self.pending_face_event = code
            await self.broadcast({"type": "face_event_status", "state": "queued", "code": code})
        await self.broadcast({"type": "task_voice", "code": code, "label": label, "live": live})
        self._task_last_voice = (code, time.monotonic())

    async def _task_move(self, vx: float, omega: float, lift: float, seconds: float,
                         sim_vx: float, sim_omega: float) -> None:
        """一段定时运动：实物走 set_chat_motion 安全脉冲（≤3s 切片），3D 由 task_motion 驱动。"""
        ms = max(0, int(seconds * 1000))
        await self.broadcast({"type": "task_motion", "vx": sim_vx, "omega": sim_omega,
                              "lift": lift, "ms": ms})
        if self._link_live():
            remaining = ms
            while remaining > 0:
                pulse = min(remaining, 2500)
                error = self.set_chat_motion(vx, 0, omega, lift, pulse)
                if error:
                    raise RuntimeError(f"运动指令被拒绝：{error}")
                await asyncio.sleep(pulse / 1000.0)
                remaining -= pulse
        else:
            await asyncio.sleep(seconds)

    def _arm_devices(self) -> list[str]:
        """执行动作的机械臂列表：优先 arm_devices，兼容旧的单臂 arm_device。"""
        cfg = dict(DEFAULT_CONFIG["task"], **(self.config.get("task") or {}))
        devices = cfg.get("arm_devices")
        if isinstance(devices, list) and devices:
            return [str(d) for d in devices]
        single = str(cfg.get("arm_device") or "")
        return [single] if single else []

    async def _invoke_store_action(self, code: str, devices: list[str]) -> str | None:
        """在多台机械臂上并发调用 Box2Robot ACT Store 动作。返回错误文本，成功为 None。"""
        b2r = Path.home() / ".kimi-code" / "skills" / "box2robot-skills" / "b2r.py"
        if not (code and devices and b2r.exists()):
            return "未配置机械臂动作或设备"

        async def run_one(device: str) -> str | None:
            try:
                proc = await asyncio.create_subprocess_exec(
                    sys.executable, str(b2r), "store", "run", code, device,
                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
                )
                out, _ = await asyncio.wait_for(proc.communicate(), timeout=30)
                text = " ".join(out.decode("utf-8", "replace").split())
                if proc.returncode == 0 and '"error"' not in text:
                    return None
                return f"{device}: {text[:120]}"
            except Exception as exc:
                return f"{device}: {exc}"

        results = await asyncio.gather(*(run_one(d) for d in devices))
        errors = [r for r in results if r]
        return "; ".join(errors) if errors else None

    async def _task_arm_action(self, code: str, devices: list[str], wait_s: float) -> None:
        """执行 Box2Robot ACT Store 预制动作；失败只提示，不中断任务。"""
        error = await self._invoke_store_action(code, devices)
        if error is None:
            await self.broadcast({"type": "task_note", "text": "机械臂动作已下发"})
        elif "未配置" in error:
            await self.broadcast({"type": "task_note", "text": "未配置机械臂动作，跳过实物执行（仅 3D 动画）"})
        else:
            await self.broadcast({"type": "task_note", "text": f"机械臂动作未执行：{error}"})
        await asyncio.sleep(max(0.0, wait_s))  # 云端异步执行，留足动作完成时间

    async def _run_quick_action(self, name: str, code: str) -> None:
        """聊天框输入预设动作名：跳过 Kimi 大模型，双臂同步直接执行，秒级响应。"""
        devices = self._arm_devices()
        await self.broadcast({"type": "chat_delta", "text": f"收到，立即在左右两臂执行「{name}」。"})
        await self.broadcast({"type": "chat_tool", "text": f"Skill: box2robot / store run {code}（{name}）× 双臂"})
        error = await self._invoke_store_action(code, devices)
        if error is None:
            await self.broadcast({"type": "chat_delta", "text": f"「{name}」已下发，两臂同步开始动作。"})
        else:
            await self.broadcast({"type": "chat_delta", "text": f"执行失败：{error}"})
        await self.broadcast({"type": "chat_done"})

    # 「向左转」「向右转 45」「右转90度」：默认 90°，角度按 turn_s=90° 比例换算时长
    TURN_RE = re.compile(r"^(向左|向右|左|右)转?\s*(\d+(?:\.\d+)?)?\s*(?:度|°)?$")

    def _parse_turn_command(self, text: str) -> tuple[float, float] | None:
        m = self.TURN_RE.match(text)
        if not m:
            return None
        sign = 1.0 if m.group(1) in ("向左", "左") else -1.0  # omega 逆时针为正
        angle = float(m.group(2)) if m.group(2) else 90.0
        if not 1.0 <= angle <= 360.0:
            return None
        return sign, angle

    async def _run_turn_command(self, sign: float, angle: float) -> None:
        cfg = dict(DEFAULT_CONFIG["task"], **(self.config.get("task") or {}))
        seconds = float(cfg["turn_s"]) * angle / 90.0
        sim_omega = sign * float(cfg["sim_turn"])  # 角速度恒定，时长按比例即可
        side = "左" if sign > 0 else "右"
        ms = int(seconds * 1000)
        await self.broadcast({"type": "chat_delta", "text": f"收到，向{side}转 {angle:g}°（约 {seconds:.1f} 秒）。"})
        await self.broadcast({"type": "chat_tool", "text": f"Skill: ra8p1-robot / move --omega {sign:g} --ms {ms}"})
        try:
            await self._task_move(0.0, sign, 0.0, seconds, 0.0, sim_omega)
            tail = "转动完成。" if self._link_live() else "（演示模式，仅 3D 展示）转动完成。"
        except Exception as exc:
            tail = f"转动失败：{exc}"
        await self.broadcast({"type": "chat_delta", "text": tail})
        await self.broadcast({"type": "chat_done"})

    async def _run_task(self) -> None:
        cfg = dict(DEFAULT_CONFIG["task"], **(self.config.get("task") or {}))
        fwd_pile = float(cfg["forward_to_pile_s"])
        turn = float(cfg["turn_s"])
        fwd_car = float(cfg["forward_to_car_s"])
        arm_s = float(cfg["arm_action_s"])
        arm_code = str(cfg.get("arm_action_code") or "")
        arm_dev = str(cfg.get("arm_device") or "")
        arm_devs = [arm_dev] if arm_dev else []  # 一键任务：单臂执行
        sim_move = float(cfg["sim_move"])
        sim_turn = float(cfg["sim_turn"])
        lift_up = float(cfg.get("lift_up_s", 1.0))  # 到桩后、取枪前，升降台升高时长
        steps = 7

        async def phase(index: int, key: str, label: str) -> None:
            await self.broadcast({
                "type": "task_status", "state": "running", "phase": key,
                "label": label, "step": index, "total": steps,
                "live": self._link_live(),
            })

        try:
            self._task_last_voice = None
            await phase(1, "start", "回到原点，任务启动")
            await self._task_say(0x01, "启动")

            await phase(2, "to_pile", "前往充电桩")
            await self._task_say(0x03, "机器人移动中")
            await self._task_move(1.0, 0.0, 0.0, fwd_pile, sim_move, 0.0)

            await phase(3, "lift", "升高升降台")
            await self._task_move(0.0, 0.0, 1.0, lift_up, 0.0, 0.0)  # 升降台上升

            await phase(4, "pick", "取电枪（机械臂）")
            await self._task_arm_action(arm_code, arm_devs, arm_s)

            await phase(5, "to_car", "前往汽车")
            await self._task_say(0x03, "机器人移动中")
            await self._task_move(0.0, -1.0, 0.0, turn, 0.0, -sim_turn)   # 右转 ~90°
            await self._task_move(1.0, 0.0, 0.0, fwd_car, sim_move, 0.0)

            await phase(6, "insert", "插枪充电（机械臂）")
            await self._task_say(0x06, "开始充电")
            await self._task_arm_action(arm_code, arm_devs, arm_s)

            await phase(7, "return", "原路返回")
            await self._task_say(0x03, "机器人移动中")
            await self._task_move(-1.0, 0.0, 0.0, fwd_car, -sim_move, 0.0)  # 后退回桩
            await self._task_move(0.0, 1.0, 0.0, turn, 0.0, sim_turn)       # 左转回正
            await self._task_move(-1.0, 0.0, 0.0, fwd_pile, -sim_move, 0.0) # 后退回原点

            await self._task_say(0x09, "任务完成")
            await self._task_say(0x0A, "安全停靠")
            await asyncio.sleep(self._voice_play_s(0x0A))  # 等最后一条语音播完再落幕
            await self.broadcast({"type": "task_status", "state": "done", "label": "任务完成，已安全停靠"})
        except asyncio.CancelledError:
            self.chat_motion = None
            self.stop_now()
            await self._task_say(0x05, "请停止", wait_prev=False)  # 中断提示立即打断当前语音
            await self.broadcast({"type": "task_status", "state": "cancelled", "label": "任务已中断，机器人已停用"})
            raise
        except Exception as exc:
            self.chat_motion = None
            self.stop_now()
            await self.broadcast({"type": "task_status", "state": "error", "label": f"任务中断：{exc}"})

    # -------------------------------------------------------------- async loops

    async def rx_pump(self) -> None:
        while True:
            item = await self.incoming.get()
            if isinstance(item, Exception):
                self.serial_error = str(item)
                await self.disconnect(safe_stop=False)
                await self.broadcast({"type": "serial_error", "message": self.serial_error})
            else:
                self._handle_telemetry(item)
                await self.broadcast({"type": "telemetry", "t": telemetry_to_dict(item)})

    async def tx_loop(self) -> None:
        while True:
            now = time.monotonic()
            slot = self._control_slot_due(now) if self.port is not None and self.sequence_synced else None
            if slot is not None:
                vx, vy, omega, lift = self._scaled_motion()
                if slot == 1 and self.pending_flags == 0 and self.pending_face_event is not None:
                    code = self.pending_face_event
                    if self._write(encode_face_event(self._next_sequence(), code)):
                        self.pending_face_event = None
                        await self.broadcast({"type": "face_event_status", "state": "sent", "code": code})
                else:
                    flags = self.pending_flags
                    if self._write(encode_control(self._next_sequence(), vx, vy, omega, lift, flags)):
                        self.pending_flags = 0
            await asyncio.sleep(TX_TICK_S)

    async def client_watchdog(self) -> None:
        """A frozen/closed tab stops producing messages: zero its motion input."""
        while True:
            if (
                self.last_client_msg_at
                and time.monotonic() - self.last_client_msg_at > CLIENT_STALE_S
                and self.motion != (0.0, 0.0, 0.0, 0.0)
            ):
                self.motion = (0.0, 0.0, 0.0, 0.0)
                self.pending_flags |= FLAG_DISARM
            await asyncio.sleep(0.2)

    # ------------------------------------------------------------------ chat

    async def handle_chat(self, text: str) -> None:
        """Relay one browser message to the Kimi Code session and back."""
        try:
            await self.chat.ask_stream(text, self.broadcast)
        except Exception as exc:
            await self.broadcast({"type": "chat_delta", "text": f"（调用 Kimi Code 失败：{exc}）"})
        finally:
            self.chat.busy = False
        await self.broadcast({"type": "chat_done"})

    # ------------------------------------------------------------------ websocket

    async def broadcast(self, message: dict) -> None:
        if not self.clients:
            return
        text = json.dumps(message, ensure_ascii=False)
        stale = []
        for ws in self.clients:
            try:
                await ws.send_str(text)
            except Exception:
                stale.append(ws)
        for ws in stale:
            self.clients.discard(ws)

    def connection_state(self) -> dict:
        if self.port is None:
            text = "未连接"
        elif self.telemetry is None:
            age = time.monotonic() - self.connected_at
            text = "已连接，但未收到遥测" if age >= 1.0 else "已连接，等待遥测"
        else:
            text = "已连接，收到遥测"
        return {
            "type": "connection",
            "connected": self.port is not None,
            "port": self.config.get("com_port", ""),
            "sequence_synced": self.sequence_synced,
            "text": self.serial_error or text,
        }

    async def handle_client_message(self, ws: web.WebSocketResponse, message: dict) -> None:
        self.last_client_msg_at = time.monotonic()
        kind = message.get("type")

        if message.get("action") == "face_event":
            await self.queue_face_event(message.get("code"))
        elif kind == "list_ports":
            await ws.send_str(json.dumps({"type": "ports", "ports": self.list_ports()}, ensure_ascii=False))
        elif kind == "connect":
            error = self.connect(str(message.get("port", "")))
            if error:
                await ws.send_str(json.dumps({"type": "error", "message": error}, ensure_ascii=False))
            await self.broadcast(self.connection_state())
        elif kind == "disconnect":
            await self.disconnect()
            await self.broadcast(self.connection_state())
        elif kind == "motion":
            self.set_motion(message.get("vx"), message.get("vy"), message.get("omega"), message.get("lift"))
        elif kind == "action":
            if str(message.get("name", "")) == "face_event":
                await self.queue_face_event(message.get("code"))
            else:
                await self.handle_action(str(message.get("name", "")))
        elif kind == "safe_stop":
            await self.cancel_one_click_task("安全停用")
            await self._send_safe_stop(3)
        elif kind == "chat":
            text = str(message.get("text", "")).strip()
            quick = (self.config.get("quick_actions") or {}).get(text)
            turn = self._parse_turn_command(text)
            if quick:
                # 预设动作名：跳过 Kimi 大模型往返，直接执行，秒级响应。
                asyncio.create_task(self._run_quick_action(text, quick))
            elif turn and not self.task_running():
                # 转向指令：向左/右转 [角度]，默认 90°，时长按比例换算。
                asyncio.create_task(self._run_turn_command(*turn))
            elif self.chat.busy:
                await self.broadcast({"type": "chat_status", "text": "上一条消息还在处理中，请稍候…"})
            elif text:
                self.chat.busy = True
                asyncio.create_task(self.handle_chat(text))
        elif kind == "chat_reset":
            self.chat.session_id = None
            await self.broadcast({"type": "chat_reset_done"})
        elif kind == "start_task":
            await self.start_one_click_task()
        elif kind == "stop_task":
            await self.cancel_one_click_task()
        elif kind == "set_config":
            if "speed_scale" in message:
                self.config["speed_scale"] = max(0.05, min(1.0, float(message["speed_scale"])))
            if "lift_speed_scale" in message:
                self.config["lift_speed_scale"] = max(0.10, min(1.0, float(message["lift_speed_scale"])))
            if "gamepad_deadzone" in message:
                self.config["gamepad_deadzone"] = max(0.0, min(0.8, float(message["gamepad_deadzone"])))
            save_config(self.config)
            await self.broadcast_config()
    async def handle_action(self, name: str) -> None:
        if name == "arm_toggle":
            # Never allow ARM before sequence sync / telemetry.
            if self.telemetry is None or not self.sequence_synced:
                return
            self.toggle_arm()
        elif name == "stop":
            await self.cancel_one_click_task("立即停用")
            self.stop_now()
        elif name == "clear_fault":
            self.pulse_flag(FLAG_CLEAR_FAULT)

    async def broadcast_config(self) -> None:
        await self.broadcast({"type": "config", "config": self.config})


async def index(_request: web.Request) -> web.StreamResponse:
    return web.FileResponse(INDEX_PATH)


# -------- HTTP API for the chat assistant's ra8p1.py CLI --------
# Same safety gating as the WebSocket path; all commands funnel into the
# single RobotBackend sender, so frame sequencing stays centralized.

async def api_state(request: web.Request) -> web.Response:
    backend: RobotBackend = request.app["backend"]
    data = backend.connection_state()
    data["telemetry"] = telemetry_to_dict(backend.telemetry) if backend.telemetry else None
    data["telemetry_fresh"] = (
        backend.telemetry is not None and time.monotonic() - backend.last_telemetry_at < 1.0
    )
    return web.json_response(data)


async def api_action(request: web.Request) -> web.Response:
    backend: RobotBackend = request.app["backend"]
    try:
        body = await request.json()
    except ValueError:
        body = {}
    name = str(body.get("name", ""))
    if name == "stop":
        backend.stop_now()
        return web.json_response({"ok": True})
    if name == "clear_fault":
        backend.pulse_flag(FLAG_CLEAR_FAULT)
        return web.json_response({"ok": True})
    if name == "arm_toggle":
        if backend.telemetry is None or not backend.sequence_synced:
            return web.json_response({"ok": False, "error": "尚未收到遥测，禁止使能"}, status=409)
        backend.toggle_arm()
        return web.json_response({"ok": True})
    if name == "face_event":
        if backend.port is None or not backend.sequence_synced:
            return web.json_response({"ok": False, "error": "串口未连接或控制序号尚未同步"}, status=409)
        try:
            code = int(body.get("code"))
        except (TypeError, ValueError):
            code = -1
        if not 0 <= code <= 0x0A:
            return web.json_response({"ok": False, "error": "事件代码必须是 0～10"}, status=400)
        backend.pending_face_event = code
        await backend.broadcast({"type": "face_event_status", "state": "queued", "code": code})
        return web.json_response({"ok": True, "queued": code})
    return web.json_response({"ok": False, "error": f"未知动作：{name}"}, status=400)


async def api_motion(request: web.Request) -> web.Response:
    backend: RobotBackend = request.app["backend"]
    try:
        body = await request.json()
    except ValueError:
        return web.json_response({"ok": False, "error": "请求体必须是 JSON"}, status=400)
    error = backend.set_chat_motion(
        body.get("vx", 0), body.get("vy", 0), body.get("omega", 0), body.get("lift", 0),
        body.get("duration_ms", 1000),
    )
    if error:
        return web.json_response({"ok": False, "error": error}, status=409)
    return web.json_response({"ok": True})


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    backend: RobotBackend = request.app["backend"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    backend.clients.add(ws)
    backend.last_client_msg_at = time.monotonic()
    await ws.send_str(json.dumps({"type": "config", "config": backend.config}, ensure_ascii=False))
    await ws.send_str(json.dumps(backend.connection_state(), ensure_ascii=False))
    await ws.send_str(
        json.dumps({"type": "ports", "ports": backend.list_ports()}, ensure_ascii=False)
    )
    try:
        async for raw in ws:
            if raw.type == web.WSMsgType.TEXT:
                try:
                    message = json.loads(raw.data)
                except ValueError:
                    continue
                if isinstance(message, dict):
                    await backend.handle_client_message(ws, message)
            elif raw.type in (web.WSMsgType.CLOSE, web.WSMsgType.ERROR):
                break
    finally:
        backend.clients.discard(ws)
        if not backend.clients and backend.port is not None:
            # Last browser left: cancel any running task, clear inputs, safe-stop.
            await backend.cancel_one_click_task("浏览器已关闭")
            await backend._send_safe_stop(3)
    return ws


async def on_shutdown(app: web.Application) -> None:
    await app["backend"].disconnect()


def main() -> None:
    backend = RobotBackend()
    app = web.Application()
    app["backend"] = backend
    app.router.add_get("/", index)
    app.router.add_get("/api/state", api_state)
    app.router.add_post("/api/action", api_action)
    app.router.add_post("/api/motion", api_motion)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_static("/assets/", BASE_DIR / "web", show_index=False)
    app.router.add_get("/favicon.ico", lambda _r: web.Response(status=204))
    app.on_shutdown.append(on_shutdown)

    async def start_tasks(app_: web.Application) -> None:
        backend.loop = asyncio.get_running_loop()

        def _quiet_exceptions(loop_: asyncio.AbstractEventLoop, context: dict) -> None:
            # Browser tabs closing abruptly flood the log with WinError 10054
            # from _ProactorBasePipeTransport; it is harmless — drop it.
            if isinstance(context.get("exception"), ConnectionResetError):
                return
            loop_.default_exception_handler(context)

        backend.loop.set_exception_handler(_quiet_exceptions)
        app_["tasks"] = [
            asyncio.create_task(backend.rx_pump()),
            asyncio.create_task(backend.tx_loop()),
            asyncio.create_task(backend.client_watchdog()),
        ]

    async def stop_tasks(app_: web.Application) -> None:
        for task in app_["tasks"]:
            task.cancel()

    app.on_startup.append(start_tasks)
    app.on_cleanup.append(stop_tasks)

    print(f"RA8P1 Web 控制器已启动： http://{HOST}:{PORT}/  （Ctrl+C 退出）")
    web.run_app(app, host=HOST, port=PORT, print=None)


if __name__ == "__main__":
    main()
