"""Tkinter keyboard/gamepad controller for the RA8P1 omni robot."""

from __future__ import annotations

import json
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Shown as a friendly GUI error on startup.
    serial = None
    list_ports = None

try:
    import pygame
except ImportError:
    pygame = None

from protocol import (
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

CONFIG_PATH = Path(__file__).with_name("controller_config.json")
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
}

# ATK-MWCC68D is half duplex. Two control slots are aligned from the
# first telemetry frame, then the rest of each 200 ms period is silent.
# A 50 ms gap avoids overflowing the module/CH340 transmit path.
CONTROL_SUPERFRAME_S = 0.200
CONTROL_TX_OFFSETS_S = (0.020, 0.070)
UI_TICK_MS = 5

FACE_EVENTS = (
    "00 待机",
    "01 启动",
    "02 欢迎回家",
    "03 机器人移动中",
    "04 请缓慢移动",
    "05 请停止",
    "06 开始充电",
    "07 充电完成",
    "08 检测到儿童",
    "09 任务完成",
    "0A 安全停靠",
)


class RobotController:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.config = self._load_config()
        self.port = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.receive_queue: queue.Queue[Telemetry | Exception] = queue.Queue()
        self.serial_error_queued = False
        self.connected_at = 0.0
        self.sequence = 0
        self.sequence_synced = False
        self.telemetry: Telemetry | None = None
        self.keys: set[str] = set()
        self.pending_flags = FLAG_DISARM
        self.pending_face_event: int | None = None
        self.window_active = True
        self.joystick = None
        self.gamepad_previous: dict[str, bool] = {}
        self.port_info = {}
        self.tx_epoch = time.monotonic()
        self.tx_cycle = 0
        self.tx_next_slot = 0

        self.port_name = tk.StringVar(value=self.config["com_port"])
        self.speed_percent = tk.DoubleVar(value=float(self.config["speed_scale"]) * 100.0)
        self.lift_speed_percent = tk.DoubleVar(value=float(self.config["lift_speed_scale"]) * 100.0)
        self.deadzone = tk.DoubleVar(value=float(self.config["gamepad_deadzone"]))
        self.connection_text = tk.StringVar(value="未连接")
        self.state_text = tk.StringVar(value="等待遥测")
        self.fault_text = tk.StringVar(value="无")
        self.link_text = tk.StringVar(value="--")
        self.lift_text = tk.StringVar(value="--")
        self.trigger_text = tk.StringVar(value="无")
        self.face_event_name = tk.StringVar(value=FACE_EVENTS[0])
        self.face_event_status = tk.StringVar(value="等待连接")
        self.servo_text = [tk.StringVar(value=f"ID{i + 1}: --") for i in range(4)]

        self._build_ui()
        self._bind_events()
        self._init_gamepad()
        self.refresh_ports()
        self.root.after(UI_TICK_MS, self._tick)

    def _load_config(self) -> dict:
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

    def _save_config(self) -> None:
        self.config["com_port"] = self.port_name.get()
        self.config["speed_scale"] = self.speed_percent.get() / 100.0
        self.config["lift_speed_scale"] = self.lift_speed_percent.get() / 100.0
        self.config["gamepad_deadzone"] = self.deadzone.get()
        CONFIG_PATH.write_text(json.dumps(self.config, ensure_ascii=False, indent=2), encoding="utf-8")

    def _build_ui(self) -> None:
        self.root.title("RA8P1 全向底盘控制器")
        self.root.geometry("720x650")
        self.root.minsize(680, 610)
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(outer, text="连接", padding=8)
        connection.pack(fill="x")
        ttk.Label(connection, text="LoRa 串口").grid(row=0, column=0, padx=4)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_name, width=18)
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=4)
        ttk.Label(connection, textvariable=self.connection_text).grid(row=0, column=4, padx=12)

        status = ttk.LabelFrame(outer, text="整机状态", padding=8)
        status.pack(fill="x", pady=(10, 0))
        labels = (
            ("状态", self.state_text),
            ("故障", self.fault_text),
            ("触发", self.trigger_text),
            ("链路", self.link_text),
            ("升降", self.lift_text),
        )
        for row, (name, variable) in enumerate(labels):
            ttk.Label(status, text=f"{name}：", width=8).grid(row=row, column=0, sticky="e")
            ttk.Label(status, textvariable=variable).grid(row=row, column=1, sticky="w")

        controls = ttk.LabelFrame(outer, text="控制", padding=8)
        controls.pack(fill="x", pady=(10, 0))
        ttk.Button(controls, text="切换使能 (Enter)", command=self.toggle_arm).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(controls, text="立即停用 (Space)", command=self.stop_now).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(controls, text="清除故障", command=lambda: self._pulse_flag(FLAG_CLEAR_FAULT)).grid(row=0, column=2, padx=4)
        ttk.Label(controls, text="底盘速度").grid(row=1, column=0, sticky="e")
        ttk.Scale(controls, from_=5, to=100, variable=self.speed_percent, orient="horizontal", length=180).grid(
            row=1, column=1, columnspan=2, sticky="w"
        )
        ttk.Label(controls, text="手柄死区").grid(row=1, column=3, sticky="e")
        ttk.Entry(controls, textvariable=self.deadzone, width=7).grid(row=1, column=4, sticky="w")
        ttk.Label(controls, text="升降速度").grid(row=2, column=0, sticky="e")
        tk.Scale(
            controls,
            from_=10,
            to=100,
            resolution=5,
            variable=self.lift_speed_percent,
            orient="horizontal",
            length=180,
            showvalue=True,
            highlightthickness=0,
        ).grid(row=2, column=1, columnspan=2, sticky="w")

        face = ttk.LabelFrame(outer, text="屏幕与扬声器", padding=8)
        face.pack(fill="x", pady=(10, 0))
        ttk.Label(face, text="事件").grid(row=0, column=0, padx=4)
        ttk.Combobox(face, textvariable=self.face_event_name, values=FACE_EVENTS, state="readonly", width=22).grid(
            row=0, column=1, padx=4
        )
        self.face_send_button = ttk.Button(face, text="发送", command=self._queue_face_event, state="disabled")
        self.face_send_button.grid(row=0, column=2, padx=4)
        ttk.Label(face, textvariable=self.face_event_status).grid(row=0, column=3, padx=12, sticky="w")

        servos = ttk.LabelFrame(outer, text="STS3215 遥测", padding=8)
        servos.pack(fill="x", pady=(10, 0))
        for index, variable in enumerate(self.servo_text):
            ttk.Label(servos, textvariable=variable).grid(row=index, column=0, sticky="w")

        help_text = (
            "键盘：W/S 前后，A/D 左右，Q/E 旋转，R/F 升降。"
            "升降无上端限位，每次上电前必须人工放到最低位；失焦、手柄断开、串口关闭时自动停用。"
        )
        ttk.Label(outer, text=help_text, foreground="#555").pack(anchor="w", pady=(10, 0))

    def _bind_events(self) -> None:
        self.root.bind_all("<KeyPress>", self._key_down)
        self.root.bind_all("<KeyRelease>", self._key_up)
        self.root.bind("<FocusOut>", lambda _event: self.root.after_idle(self._check_focus))
        self.root.bind("<FocusIn>", lambda _event: setattr(self, "window_active", True))
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _init_gamepad(self) -> None:
        if pygame is None:
            return
        pygame.init()
        pygame.joystick.init()
        self._refresh_gamepad()

    def _refresh_gamepad(self) -> None:
        if pygame is None or pygame.joystick.get_count() == 0:
            if self.joystick is not None:
                self.joystick = None
                self.stop_now()
            return
        if self.joystick is None:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.gamepad_previous.clear()

    def refresh_ports(self) -> None:
        if list_ports is None:
            return
        ports = list(list_ports.comports())
        self.port_info = {item.device: item for item in ports}
        names = [item.device for item in ports]
        usable_names = [item.device for item in ports if item.vid != 0x1366]
        self.port_combo["values"] = names
        if self.port_name.get() not in usable_names:
            self.port_name.set(usable_names[0] if usable_names else "")

    def toggle_connection(self) -> None:
        if self.port is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self) -> None:
        if serial is None:
            messagebox.showerror("缺少依赖", "请先运行：pip install -r requirements.txt")
            return
        selected_port = self.port_name.get().strip()
        if not selected_port:
            messagebox.showwarning("未选择串口", "请连接电脑端 LoRa/USB-TTL，点击刷新并选择新出现的 COM 口。")
            return
        port_info = self.port_info.get(selected_port)
        if port_info is not None and port_info.vid == 0x1366:
            messagebox.showwarning("串口选择错误", f"{selected_port} 是 J-Link 调试器，不是电脑端 LoRa 串口。")
            return
        try:
            self.port = serial.Serial(selected_port, int(self.config["baud"]), timeout=0.05, write_timeout=0.1)
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            self.port = None
            return
        self.reader_stop.clear()
        self.serial_error_queued = False
        self.connected_at = time.monotonic()
        self.telemetry = None
        self.sequence_synced = False
        self.pending_flags |= FLAG_DISARM
        self.pending_face_event = None
        self.face_send_button.configure(state="disabled")
        self.face_event_status.set("等待遥测同步")
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connection_text.set("已连接")
        self.connect_button.configure(text="断开")
        self.tx_epoch = time.monotonic()
        self.tx_cycle = 0
        self.tx_next_slot = 0
        self._save_config()

    def disconnect(self, safe_stop: bool = True) -> None:
        if safe_stop:
            self._send_safe_stop(3)
        self.reader_stop.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=0.3)
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.sequence_synced = False
        self.pending_face_event = None
        self.face_send_button.configure(state="disabled")
        self.face_event_status.set("等待连接")
        self.connection_text.set("未连接")
        self.connect_button.configure(text="连接")

    def _reader_loop(self) -> None:
        parser = FrameParser()
        while not self.reader_stop.is_set() and self.port is not None:
            try:
                data = self.port.read(256)
                for frame in parser.feed(data):
                    if frame.message_type == MSG_TELEMETRY:
                        self.receive_queue.put(decode_telemetry(frame.payload))
            except Exception as exc:
                if not self.serial_error_queued:
                    self.serial_error_queued = True
                    self.receive_queue.put(exc)
                break

    def _next_sequence(self) -> int:
        value = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        return value

    def _write(self, data: bytes) -> bool:
        if self.port is None or not self.port.is_open:
            return False
        try:
            self.port.write(data)
            return True
        except Exception as exc:
            if not self.serial_error_queued:
                self.serial_error_queued = True
                self.receive_queue.put(exc)
            return False

    def _send_safe_stop(self, count: int) -> None:
        self.keys.clear()
        for _ in range(count):
            self._write(encode_control(self._next_sequence(), 0, 0, 0, 0, FLAG_DISARM))
            time.sleep(0.05)

    def toggle_arm(self) -> None:
        active = self.telemetry is not None and self.telemetry.state in (1, 3, 5, 6)
        self._pulse_flag(FLAG_DISARM if active else FLAG_ARM)

    def stop_now(self) -> None:
        self.keys.clear()
        self.pending_flags |= FLAG_DISARM
        self._write(encode_control(self._next_sequence(), 0, 0, 0, 0, FLAG_DISARM))

    def _pulse_flag(self, flag: int) -> None:
        self.pending_flags |= flag

    def _queue_face_event(self) -> None:
        try:
            code = FACE_EVENTS.index(self.face_event_name.get())
        except ValueError:
            return
        self._queue_face_event_code(code)

    def _queue_face_event_code(self, code: int) -> None:
        if self.port is None or not self.sequence_synced:
            self.face_event_status.set("串口未连接或序号未同步")
            return
        self.pending_face_event = code
        self.face_event_status.set(f"0x{code:02X} 已排队")

    def _key_down(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key == "return":
            self.toggle_arm()
        elif key == "space":
            self.stop_now()
        elif key in {"w", "a", "s", "d", "q", "e", "r", "f"}:
            self.keys.add(key)

    def _key_up(self, event: tk.Event) -> None:
        self.keys.discard(event.keysym.lower())

    def _check_focus(self) -> None:
        if self.root.focus_displayof() is None:
            self.window_active = False
            self._send_safe_stop(3)

    @staticmethod
    def _deadzone(value: float, zone: float) -> float:
        if abs(value) <= zone:
            return 0.0
        return (abs(value) - zone) / (1.0 - zone) * (1.0 if value > 0 else -1.0)

    def _gamepad_motion(self) -> tuple[float, float, float, float]:
        if pygame is None:
            return 0.0, 0.0, 0.0, 0.0
        pygame.event.pump()
        self._refresh_gamepad()
        if self.joystick is None:
            return 0.0, 0.0, 0.0, 0.0
        axes = self.config["gamepad_axes"]
        zone = max(0.0, min(0.8, float(self.deadzone.get())))
        try:
            vy = -self._deadzone(self.joystick.get_axis(int(axes["left_x"])), zone)
            vx = -self._deadzone(self.joystick.get_axis(int(axes["left_y"])), zone)
            omega = -self._deadzone(self.joystick.get_axis(int(axes["right_x"])), zone)
            buttons = self.config["gamepad_buttons"]
            lift = float(self.joystick.get_button(int(buttons["right_shoulder"]))) - float(
                self.joystick.get_button(int(buttons["left_shoulder"]))
            )
            for name in ("start", "stop"):
                number = int(buttons[name])
                pressed = bool(self.joystick.get_button(number))
                if pressed and not self.gamepad_previous.get(name, False):
                    if name == "start":
                        self.toggle_arm()
                    else:
                        self.stop_now()
                self.gamepad_previous[name] = pressed
            for name, code in (
                ("face_startup", 0x01),
                ("face_charge_start", 0x06),
                ("face_charge_complete", 0x07),
                ("face_safe_dock", 0x0A),
            ):
                number = int(buttons[name])
                pressed = bool(self.joystick.get_button(number))
                if pressed and not self.gamepad_previous.get(name, False):
                    self._queue_face_event_code(code)
                self.gamepad_previous[name] = pressed
            return vx, vy, omega, lift
        except (IndexError, pygame.error):
            self.joystick = None
            self.stop_now()
            return 0.0, 0.0, 0.0, 0.0

    def _motion(self) -> tuple[int, int, int, int]:
        vx = float("w" in self.keys) - float("s" in self.keys)
        vy = float("a" in self.keys) - float("d" in self.keys)
        omega = float("q" in self.keys) - float("e" in self.keys)
        lift = float("r" in self.keys) - float("f" in self.keys)
        gx, gy, go, gl = self._gamepad_motion()
        vx, vy, omega, lift = (max(-1.0, min(1.0, a + b)) for a, b in ((vx, gx), (vy, gy), (omega, go), (lift, gl)))
        chassis_scale = max(0.05, min(1.0, self.speed_percent.get() / 100.0))
        lift_scale = max(0.10, min(1.0, self.lift_speed_percent.get() / 100.0))
        return (
            int(round(vx * chassis_scale * 1000.0)),
            int(round(vy * chassis_scale * 1000.0)),
            int(round(omega * chassis_scale * 1000.0)),
            int(round(lift * lift_scale * 1000.0)),
        )

    def _update_telemetry(self, telemetry: Telemetry) -> None:
        if not self.sequence_synced:
            # A restarted PC app begins with a new local sequence counter, while
            # the running MCU still remembers the previous session's counter.
            # Continue after the MCU's last accepted value so reconnects are not
            # rejected as duplicate/out-of-order control frames.
            self.sequence = (telemetry.last_control_sequence + 1) & 0xFFFF
            self.sequence_synced = True
            self.pending_flags |= FLAG_DISARM
            self.tx_epoch = time.monotonic()
            self.tx_cycle = 0
            self.tx_next_slot = 0
            self.face_send_button.configure(state="normal")
        self.telemetry = telemetry
        self.connection_text.set("已连接，收到遥测")
        self.state_text.set(telemetry.state_name)
        self.fault_text.set(telemetry.fault_text)
        self.trigger_text.set(telemetry.fault_snapshot.fault_text)
        self.link_text.set(f"{telemetry.link_age_ms} ms，帧序号 {telemetry.last_control_sequence}")
        self.lift_text.set(
            f"位置 {telemetry.lift_position} / 目标 {telemetry.lift_target}，"
            f"零点={'有效' if telemetry.homed else '无效'}，下限开关=未使用"
        )
        for index, servo in enumerate(telemetry.servos):
            temperature_limit = f"{servo.temperature_limit_c} °C" if servo.temperature_limit_c is not None else "--"
            self.servo_text[index].set(
                f"ID{index + 1}: {'在线' if servo.online else '掉线'}  {servo.voltage_v:.1f} V  "
                f"{servo.temperature_c} °C / 上限 {temperature_limit}  "
                f"位置 {servo.position}  速度 {servo.speed}  电流 {servo.current}  "
                f"{servo.fault_text}"
            )

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

    def _tick(self) -> None:
        try:
            while True:
                item = self.receive_queue.get_nowait()
                if isinstance(item, Exception):
                    error_text = f"串口错误：{item}"
                    self.disconnect(safe_stop=False)
                    self.connection_text.set(error_text)
                    break
                else:
                    self._update_telemetry(item)
        except queue.Empty:
            pass

        now = time.monotonic()
        if self.port is not None:
            if self.telemetry is None and (time.monotonic() - self.connected_at) >= 1.0:
                self.connection_text.set("已连接，但未收到遥测")
            slot = self._control_slot_due(now) if self.sequence_synced else None
            if slot is not None:
                motion = self._motion() if self.window_active else (0, 0, 0, 0)
                if slot == 1 and self.pending_flags == 0 and self.pending_face_event is not None:
                    code = self.pending_face_event
                    if self._write(encode_face_event(self._next_sequence(), code)):
                        self.pending_face_event = None
                        self.face_event_status.set(f"0x{code:02X} 已发送至 RA8P1")
                else:
                    flags = self.pending_flags
                    if self._write(encode_control(self._next_sequence(), *motion, flags)):
                        self.pending_flags = 0
        self.root.after(UI_TICK_MS, self._tick)

    def close(self) -> None:
        self._save_config()
        self.disconnect()
        if pygame is not None:
            pygame.quit()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    RobotController(root)
    root.mainloop()


if __name__ == "__main__":
    main()
