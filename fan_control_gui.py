# -*- coding: utf-8 -*-
"""
fan_control_gui.py — RoboMaster C板 风机控制 图形化界面
依赖：Python 3.7+ 标准库（tkinter / pyserial）
若未安装 pyserial：pip install pyserial
"""

import re
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("需要 pyserial：  pip install pyserial")
    sys.exit(1)


BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]


class FanControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("C板 风机控制  v1.0")
        self.root.geometry("720x520")
        self.root.minsize(680, 480)

        self.ser = None
        self._rx_thread_running = False
        self._connected = False

        self._build_ui()
        self._refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # UI 构建
    # ------------------------------------------------------------------
    def _build_ui(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass

        pad = {"padx": 8, "pady": 4}

        # --- 端口连接区 ------------------------------------------------
        conn_frame = ttk.LabelFrame(self.root, text="串口连接")
        conn_frame.pack(fill="x", **pad)

        ttk.Label(conn_frame, text="COM 口:").grid(row=0, column=0, padx=6, pady=6, sticky="e")
        self.cmb_port = ttk.Combobox(conn_frame, width=22, state="readonly")
        self.cmb_port.grid(row=0, column=1, padx=4, pady=6)

        ttk.Label(conn_frame, text="波特率:").grid(row=0, column=2, padx=6, pady=6, sticky="e")
        self.cmb_baud = ttk.Combobox(conn_frame, values=BAUDRATES, width=10, state="readonly")
        self.cmb_baud.current(4)  # 默认 115200
        self.cmb_baud.grid(row=0, column=3, padx=4, pady=6)

        self.btn_refresh = ttk.Button(conn_frame, text="刷新端口", command=self._refresh_ports)
        self.btn_refresh.grid(row=0, column=4, padx=6, pady=6)

        self.btn_connect = ttk.Button(conn_frame, text="连接", width=10, command=self._toggle_connect)
        self.btn_connect.grid(row=0, column=5, padx=6, pady=6)

        self.lbl_conn_status = ttk.Label(conn_frame, text="● 未连接", foreground="gray")
        self.lbl_conn_status.grid(row=0, column=6, padx=12, pady=6, sticky="w")

        # --- 控制区 ----------------------------------------------------
        ctrl_frame = ttk.LabelFrame(self.root, text="风机控制")
        ctrl_frame.pack(fill="x", **pad)

        # 第一行：启动/停止
        self.btn_start = ttk.Button(ctrl_frame, text="▶ 启动 (START)", width=18,
                                     state="disabled", command=lambda: self._send_cmd("START"))
        self.btn_start.grid(row=0, column=0, padx=8, pady=8)

        self.btn_stop = ttk.Button(ctrl_frame, text="■ 停止 (STOP)", width=18,
                                    state="disabled", command=lambda: self._send_cmd("STOP"))
        self.btn_stop.grid(row=0, column=1, padx=8, pady=8)

        self.btn_status = ttk.Button(ctrl_frame, text="⟳ 刷新状态", width=14,
                                      state="disabled", command=lambda: self._send_cmd("STATUS"))
        self.btn_status.grid(row=0, column=2, padx=8, pady=8)

        # 第二行：占空比设置
        duty_row = ttk.Frame(ctrl_frame)
        duty_row.grid(row=1, column=0, columnspan=3, padx=6, pady=6, sticky="ew")

        ttk.Label(duty_row, text="目标占空比:").pack(side="left", padx=4)

        self.slider = tk.Scale(duty_row, from_=0, to=100, orient="horizontal",
                                length=420, resolution=1, command=self._on_slider,
                                state="disabled", showvalue=0)
        self.slider.pack(side="left", padx=6, fill="x", expand=True)

        self.lbl_duty = ttk.Label(duty_row, text="50 %", width=7,
                                   font=("Consolas", 12, "bold"))
        self.lbl_duty.pack(side="left", padx=4)
        self.slider.set(50)

        self.btn_send_duty = ttk.Button(duty_row, text="设置", width=6,
                                         state="disabled", command=self._send_current_duty)
        self.btn_send_duty.pack(side="left", padx=6)

        # 第三行：快捷占空比按钮
        preset_row = ttk.Frame(ctrl_frame)
        preset_row.grid(row=2, column=0, columnspan=3, padx=6, pady=(0, 8), sticky="ew")
        ttk.Label(preset_row, text="快捷档位:").pack(side="left", padx=4)
        for pct in (10, 25, 30, 50, 70, 80, 90, 100):
            btn = ttk.Button(preset_row, text=f"{pct}%", width=5,
                              state="disabled",
                              command=lambda v=pct: self._preset_duty(v))
            btn._preset_pct = pct
            btn.pack(side="left", padx=3)
            setattr(self, f"btn_preset_{pct}", btn)

        # --- 状态显示区 ------------------------------------------------
        status_frame = ttk.LabelFrame(self.root, text="当前状态")
        status_frame.pack(fill="x", **pad)

        self.lbl_state = ttk.Label(status_frame, text="状态: —", width=20,
                                    font=("Consolas", 11))
        self.lbl_state.grid(row=0, column=0, padx=16, pady=8, sticky="w")

        self.lbl_current = ttk.Label(status_frame, text="当前占空比: —", width=20,
                                      font=("Consolas", 11))
        self.lbl_current.grid(row=0, column=1, padx=16, pady=8, sticky="w")

        self.lbl_target = ttk.Label(status_frame, text="目标占空比: —", width=20,
                                     font=("Consolas", 11))
        self.lbl_target.grid(row=0, column=2, padx=16, pady=8, sticky="w")

        self.progress = ttk.Progressbar(status_frame, maximum=100, length=380)
        self.progress.grid(row=1, column=0, columnspan=3, padx=16, pady=(0, 10), sticky="w")

        # --- 日志/命令区 -----------------------------------------------
        log_frame = ttk.LabelFrame(self.root, text="串口日志（底部可手动发命令）")
        log_frame.pack(fill="both", expand=True, **pad)

        self.txt_log = scrolledtext.ScrolledText(log_frame, height=9,
                                                  font=("Consolas", 9), state="disabled")
        self.txt_log.pack(fill="both", expand=True, padx=6, pady=(6, 4))

        send_row = ttk.Frame(log_frame)
        send_row.pack(fill="x", padx=6, pady=(0, 6))
        self.ent_cmd = ttk.Entry(send_row)
        self.ent_cmd.pack(side="left", fill="x", expand=True)
        self.ent_cmd.bind("<Return>", lambda e: self._send_manual())
        self.btn_send_cmd = ttk.Button(send_row, text="发送", width=8,
                                        state="disabled", command=self._send_manual)
        self.btn_send_cmd.pack(side="left", padx=6)

    # ------------------------------------------------------------------
    # 串口连接
    # ------------------------------------------------------------------
    def _refresh_ports(self):
        ports = [f"{p.device}  {p.description}" for p in serial.tools.list_ports.comports()]
        self.cmb_port["values"] = ports
        if ports and not self.cmb_port.get():
            # 优先选 STMicroelectronics Virtual COM Port 或 USB Serial
            for p in ports:
                if "STMicro" in p or "Virtual COM" in p or "USB Serial" in p or "COM3" in p:
                    self.cmb_port.set(p)
                    return
            self.cmb_port.set(ports[0])

    def _toggle_connect(self):
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        sel = self.cmb_port.get()
        if not sel:
            messagebox.showwarning("提示", "请先选择 COM 口")
            return
        port_name = sel.split()[0]
        try:
            self.ser = serial.Serial(port_name, int(self.cmb_baud.get()),
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     bytesize=serial.EIGHTBITS,
                                     timeout=0.2)
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            return

        self._connected = True
        self._rx_thread_running = True
        threading.Thread(target=self._rx_loop, daemon=True).start()

        self.btn_connect.config(text="断开")
        self.lbl_conn_status.config(text="● 已连接  " + port_name, foreground="green")
        self._set_controls(True)
        self._log(f"[已连接] {port_name}")

        # 自动查一次状态
        time.sleep(0.2)
        self._send_cmd("STATUS")

    def _disconnect(self):
        self._rx_thread_running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._connected = False

        self.btn_connect.config(text="连接")
        self.lbl_conn_status.config(text="● 未连接", foreground="gray")
        self._set_controls(False)
        self._log("[已断开]")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    # ------------------------------------------------------------------
    # 控制按钮启用状态
    # ------------------------------------------------------------------
    def _set_controls(self, enabled):
        state = "normal" if enabled else "disabled"
        for w in (self.btn_start, self.btn_stop, self.btn_status,
                   self.btn_send_duty, self.btn_send_cmd):
            w.config(state=state)
        self.slider.config(state=("normal" if enabled else "disabled"))
        for pct in (10, 25, 30, 50, 70, 80, 90, 100):
            getattr(self, f"btn_preset_{pct}").config(state=state)

    # ------------------------------------------------------------------
    # 发送命令
    # ------------------------------------------------------------------
    def _send_cmd(self, cmd):
        if not (self.ser and self.ser.is_open):
            return
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
            self._log(f"→ {cmd}")
        except Exception as e:
            self._log(f"[发送失败] {e}")

    def _send_manual(self):
        cmd = self.ent_cmd.get().strip()
        if not cmd:
            return
        self.ent_cmd.delete(0, "end")
        self._send_cmd(cmd)

    def _on_slider(self, val):
        self.lbl_duty.config(text=f"{int(float(val))} %")

    def _preset_duty(self, pct):
        self.slider.set(pct)
        self._send_current_duty()

    def _send_current_duty(self):
        pct = int(float(self.slider.get()))
        self._send_cmd(f"S{pct}")

    # ------------------------------------------------------------------
    # 串口接收线程
    # ------------------------------------------------------------------
    def _rx_loop(self):
        buf = b""
        while self._rx_thread_running and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(64)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.rstrip(b"\r")
                try:
                    text = line.decode("utf-8", errors="replace")
                except Exception:
                    text = repr(line)
                self.root.after(0, self._on_reply, text)

    def _on_reply(self, text):
        self._log(f"← {text}")
        # 解析 STATUS 行：STATE=RUN DUTY=80 TARGET=80
        m = re.match(r"STATE=(\S+)\s+DUTY=(\d+)\s+TARGET=(\d+)", text)
        if m:
            state, duty, target = m.group(1), int(m.group(2)), int(m.group(3))
            self.lbl_state.config(text=f"状态: {state}")
            self.lbl_current.config(text=f"当前占空比: {duty} %")
            self.lbl_target.config(text=f"目标占空比: {target} %")
            self.progress.config(value=duty)
            return
        # 解析 OK Sxx TARGET=xx
        m = re.match(r"OK S(\d+) TARGET=(\d+)", text)
        if m:
            self.lbl_target.config(text=f"目标占空比: {m.group(2)} %")
            return
        # 解析 OK START / OK STOP
        if text.startswith("OK START"):
            self.lbl_state.config(text="状态: 启动中")
        elif text.startswith("OK STOP"):
            self.lbl_state.config(text="状态: 停止中")

    # ------------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------------
    def _log(self, msg):
        self.txt_log.config(state="normal")
        self.txt_log.insert("end", time.strftime("%H:%M:%S ") + msg + "\n")
        self.txt_log.see("end")
        self.txt_log.config(state="disabled")


def main():
    root = tk.Tk()
    FanControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
