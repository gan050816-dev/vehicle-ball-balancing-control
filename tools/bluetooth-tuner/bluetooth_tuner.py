"""Windows GUI for MSPM0G3507 task-3 Bluetooth tuning."""

from __future__ import annotations

import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports

from bluetooth_protocol import (
    CMD_HELLO,
    CMD_READ_PARAM,
    CMD_SET_STREAM,
    CMD_WRITE_PARAM,
    Frame,
    FrameParser,
    RSP_ACK,
    RSP_ERROR,
    RSP_INFO,
    RSP_PARAM,
    RSP_STATUS,
    STATUS_OK,
    decode_status,
    encode_frame,
)

BAUD_RATE = 9600

# id: (name, display scale from raw integer, unit)
PARAMETERS = {
    1: ("Kp", 0.001, "deg/mm"),
    2: ("Kd", 0.001, "deg·s/mm"),
    3: ("启动角", 0.01, "deg"),
    4: ("PD接管位置", 0.1, "mm"),
    5: ("预测时间", 1.0, "ms"),
    6: ("助推进入速度", 0.1, "mm/s"),
    7: ("助推退出速度", 0.1, "mm/s"),
    8: ("最小角", 0.01, "deg"),
    9: ("最大角", 0.01, "deg"),
    10: ("步进速度", 1.0, "RPM"),
    11: ("步进加速度档", 1.0, "level"),
}

APP_STATES = ["HOME", "LEVEL", "MENU", "PLACE", "CAPTURE", "READY", "RUN", "DONE", "FAULT"]
PHASES = ["NOT_STARTED", "POS_DRIVE", "POS_APPROACH", "NEG_TRACK", "LEVEL_TIMEOUT"]
STEPPER_STATES = ["STOPPED", "LEVELING", "READY", "FAULT"]


class SerialTransport:
    def __init__(self, event_queue: queue.Queue) -> None:
        self.event_queue = event_queue
        self.port: serial.Serial | None = None
        self.parser = FrameParser()
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.reader_thread: threading.Thread | None = None

    def open(self, port_name: str) -> None:
        self.parser = FrameParser()
        self.port = serial.Serial(port_name, BAUD_RATE, timeout=0.05, write_timeout=0.5)
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.port is not None:
            self.port.close()
            self.port = None
        if self.reader_thread is not None and self.reader_thread is not threading.current_thread():
            self.reader_thread.join(timeout=0.2)
        self.reader_thread = None

    def send(self, frame: bytes) -> None:
        if self.port is None:
            raise RuntimeError("serial port is not connected")
        with self.write_lock:
            self.port.write(frame)

    def _reader(self) -> None:
        try:
            while not self.stop_event.is_set() and self.port is not None:
                data = self.port.read(self.port.in_waiting or 1)
                for frame in self.parser.feed(data):
                    self.event_queue.put(("frame", frame))
        except Exception as exc:  # presented in the UI thread
            self.event_queue.put(("error", str(exc)))


class TunerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("TI任务三 蓝牙调参")
        self.events: queue.Queue = queue.Queue()
        self.transport = SerialTransport(self.events)
        self.sequence = 0
        self.requests: list[tuple[int, bytes, int]] = []
        self.outstanding: dict | None = None
        self.param_entries: dict[int, ttk.Entry] = {}
        self.connected = False
        self._build_ui()
        self.refresh_ports()
        self.root.after(20, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.grid(sticky="ew")
        ttk.Label(top, text="蓝牙COM口").grid(row=0, column=0, padx=4)
        self.port_box = ttk.Combobox(top, width=20, state="readonly")
        self.port_box.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(top, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=4)
        self.connection_label = ttk.Label(top, text="未连接")
        self.connection_label.grid(row=0, column=4, padx=8)

        params = ttk.LabelFrame(self.root, text="任务三运行参数（复位后恢复固件默认值）", padding=8)
        params.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)
        for column, title in enumerate(("参数", "当前/目标值", "单位", "操作")):
            ttk.Label(params, text=title).grid(row=0, column=column, padx=4, pady=2)
        for row, (param_id, (name, _, unit)) in enumerate(PARAMETERS.items(), start=1):
            ttk.Label(params, text=name).grid(row=row, column=0, sticky="w", padx=4)
            entry = ttk.Entry(params, width=14)
            entry.grid(row=row, column=1, padx=4, pady=2)
            self.param_entries[param_id] = entry
            ttk.Label(params, text=unit).grid(row=row, column=2, sticky="w", padx=4)
            buttons = ttk.Frame(params)
            buttons.grid(row=row, column=3)
            ttk.Button(buttons, text="读", width=5, command=lambda p=param_id: self.read_param(p)).pack(side="left")
            ttk.Button(buttons, text="写", width=5, command=lambda p=param_id: self.write_param(p)).pack(side="left")
        ttk.Button(params, text="读取全部参数", command=self.read_all).grid(row=len(PARAMETERS) + 1, column=0, columnspan=4, pady=6)

        status_frame = ttk.LabelFrame(self.root, text="当前状态（100 ms）", padding=8)
        status_frame.grid(row=1, column=1, sticky="nsew", padx=8, pady=4)
        self.status_text = tk.Text(status_frame, width=55, height=22, state="disabled")
        self.status_text.pack(fill="both", expand=True)
        self.log_text = tk.Text(self.root, width=110, height=8, state="disabled")
        self.log_text.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=8, pady=8)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and not self.port_box.get():
            self.port_box.current(0)

    def toggle_connection(self) -> None:
        if self.connected:
            self.transport.close()
            self.connected = False
            self.outstanding = None
            self.requests.clear()
            self.connect_button.config(text="连接")
            self.connection_label.config(text="未连接")
            return
        port_name = self.port_box.get()
        if not port_name:
            messagebox.showerror("连接失败", "请选择蓝牙COM口")
            return
        try:
            self.transport.open(port_name)
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        self.connected = True
        self.connect_button.config(text="断开")
        self.connection_label.config(text=f"{port_name} @ {BAUD_RATE}")
        self._enqueue(CMD_HELLO, b"", RSP_INFO)

    def _enqueue(self, command: int, payload: bytes, expected: int) -> None:
        if not self.connected:
            self._log("未连接")
            return
        self.requests.append((command, payload, expected))
        self._pump_requests()

    def _pump_requests(self) -> None:
        if self.outstanding is not None or not self.requests or not self.connected:
            return
        command, payload, expected = self.requests.pop(0)
        self.sequence = (self.sequence + 1) & 0xFF
        encoded = encode_frame(command, self.sequence, payload)
        try:
            self.transport.send(encoded)
        except Exception as exc:
            self._log(f"发送失败: {exc}")
            return
        self.outstanding = {
            "sequence": self.sequence, "expected": expected, "frame": encoded,
            "sent": time.monotonic(), "retries": 0,
        }

    def read_param(self, param_id: int) -> None:
        self._enqueue(CMD_READ_PARAM, bytes((param_id,)), RSP_PARAM)

    def read_all(self) -> None:
        for param_id in PARAMETERS:
            self.read_param(param_id)

    def write_param(self, param_id: int) -> None:
        _, scale, _ = PARAMETERS[param_id]
        try:
            raw_value = round(float(self.param_entries[param_id].get()) / scale)
        except ValueError:
            messagebox.showerror("参数错误", "请输入数字")
            return
        self._enqueue(CMD_WRITE_PARAM, bytes((param_id,)) + struct.pack("<i", raw_value), RSP_PARAM)

    def _enable_stream(self) -> None:
        self._enqueue(CMD_SET_STREAM, bytes((1, 100, 0)), RSP_ACK)

    def _handle_frame(self, frame: Frame) -> None:
        if frame.command == RSP_STATUS:
            try:
                self._show_status(decode_status(frame.payload))
            except ValueError as exc:
                self._log(str(exc))
        elif frame.command == RSP_INFO:
            if len(frame.payload) == 4 and frame.payload[0] == STATUS_OK:
                self._log(f"握手成功：协议v{frame.payload[1]}，参数{frame.payload[2]}项，RAM临时参数")
                self.read_all()
                self._enable_stream()
        elif frame.command == RSP_PARAM and len(frame.payload) == 6:
            status, param_id = frame.payload[:2]
            raw_value = struct.unpack_from("<i", frame.payload, 2)[0]
            if status == STATUS_OK and param_id in PARAMETERS:
                _, scale, _ = PARAMETERS[param_id]
                value = raw_value * scale
                entry = self.param_entries[param_id]
                entry.delete(0, "end")
                entry.insert(0, f"{value:g}")
                self._log(f"参数{param_id}已确认，实际值={value:g}")
            else:
                self._log(f"参数{param_id}被拒绝，状态={status}，保持原值={raw_value}")
        elif frame.command == RSP_ACK:
            self._log("状态流已启用")
        elif frame.command == RSP_ERROR and len(frame.payload) == 2:
            self._log(f"从机错误：命令=0x{frame.payload[0]:02X} 状态={frame.payload[1]}")

        if self.outstanding is not None and frame.sequence == self.outstanding["sequence"]:
            if frame.command in (self.outstanding["expected"], RSP_ERROR):
                self.outstanding = None
                self._pump_requests()

    def _show_status(self, status: dict[str, int]) -> None:
        app = APP_STATES[status["app_state"]] if status["app_state"] < len(APP_STATES) else str(status["app_state"])
        phase = PHASES[status["phase"]] if status["phase"] < len(PHASES) else str(status["phase"])
        stepper = STEPPER_STATES[status["stepper_status"]] if status["stepper_status"] < len(STEPPER_STATES) else str(status["stepper_status"])
        flag_names = [name for bit, name in ((1, "视觉"), (2, "速度"), (4, "角命令"), (8, "跟踪"), (16, "+50到达"), (32, "-50到达"), (64, "RX溢出")) if status["flags"] & bit]
        text = (
            f"应用: {app}\n阶段: {phase}\n步进: {stepper}\n标志: {', '.join(flag_names) or '-'}\n\n"
            f"运行时间: {status['elapsed_ms']} ms\n"
            f"球原始X: {status['raw_x'] / 10:.1f} mm\n"
            f"球滤波X: {status['filtered_x'] / 10:.1f} mm\n"
            f"球速度: {status['velocity'] / 10:.1f} mm/s\n"
            f"目标相对角: {status['command_angle'] / 100:.2f} deg\n"
            f"X42命令/测量: {status['commanded_rack'] / 10:.1f}/{status['measured_rack'] / 10:.1f} mm代理\n\n"
            f"P: {status['positive_time']} ms  V+: {status['positive_velocity'] / 10:.1f} mm/s\n"
            f"N: {status['negative_time']} ms  V-: {status['negative_velocity'] / 10:.1f} mm/s\n"
            f"协议错误: {status['protocol_errors']}  RX溢出: {status['rx_overflows']}"
        )
        self.status_text.config(state="normal")
        self.status_text.delete("1.0", "end")
        self.status_text.insert("1.0", text)
        self.status_text.config(state="disabled")

    def _log(self, message: str) -> None:
        self.log_text.config(state="normal")
        self.log_text.insert("end", time.strftime("%H:%M:%S ") + message + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _poll(self) -> None:
        try:
            while True:
                event, value = self.events.get_nowait()
                if event == "frame":
                    self._handle_frame(value)
                else:
                    self._log(f"串口错误: {value}")
        except queue.Empty:
            pass
        if self.outstanding is not None and time.monotonic() - self.outstanding["sent"] > 0.6:
            if self.outstanding["retries"] < 2:
                try:
                    self.transport.send(self.outstanding["frame"])
                    self.outstanding["retries"] += 1
                    self.outstanding["sent"] = time.monotonic()
                except Exception as exc:
                    self._log(f"重发失败: {exc}")
                    self.outstanding = None
            else:
                self._log("请求超时，已重试2次")
                self.outstanding = None
                self._pump_requests()
        self.root.after(20, self._poll)

    def _on_close(self) -> None:
        self.transport.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    TunerApp(root)
    root.mainloop()
