#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双 MCU 物料分拣系统 - 上位机监控 GUI (PC 侧)
=============================================================================
读取 F407 Modbus 网关从站(addr=0x02)的 15 个保持寄存器, 实时显示系统状态 /
电机 PWM / 传感 / 计数 / POST 自检 / 黑匣子事件, 并下发 启动-停止-复位 与
目标速度指令。

技术栈: tkinter (标准库, 零依赖) + pyserial (用户已装)。
架构: 三层解耦, 与嵌入式端 FreeRTOS 三任务解耦思想对应 ——
  - 通信层  ModbusThread : 后台线程, 500ms 轮询读 + 自动重连, 不阻塞 UI
  - 数据层  regmap_defs  : 寄存器/位定义唯一真源 (与固件 mb_regmap.h 同步)
  - UI 层    SorterGUI    : 只负责显示 + 发指令, 经 queue 与通信层解耦

运行:  python sorter_gui.py              # 默认 COM14 115200
       python sorter_gui.py --com COM3   # 指定串口号
依赖:  pip install pyserial
"""
import sys
import time
import json
import csv
import threading
import queue
from collections import deque
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext, messagebox, filedialog
except ImportError:
    raise SystemExit("缺少 tkinter (Python 标准库 GUI)。请使用完整版 Python 或安装 Tcl/Tk 后重试。")

try:
    import serial
except ImportError:
    raise SystemExit("缺少 pyserial, 请先安装: pip install pyserial")

# matplotlib 可选 (SENSE 历史曲线)。缺库不阻塞 GUI, 曲线区自动隐藏。
try:
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

# ---- 配置持久化 (#47): ~/.sorter_gui.json 保存 串口/波特/窗口位置 ----
CONFIG_PATH = Path.home() / ".sorter_gui.json"


def load_config():
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except Exception:
        pass


# 事件码 → 日志级别 (黑匣子染色, #48)
EVENT_LEVEL = {
    0: "INFO",  # NONE
    1: "ERROR",  # FAULT(故障)
    2: "WARN",   # F103掉线
    3: "INFO",   # 故障消除
    4: "WARN",   # PC失联
    5: "INFO",   # PC恢复
    6: "INFO",   # F103上线
    7: "ERROR",  # 任务超时(看门狗)
}

from regmap_defs import (
    GW_ADDR, GW_REG_COUNT, BAUD_DEFAULT, REG_NAMES,
    SYS_STATUS_BITS, MOTOR_STATUS_BITS, FAULT_BITS, POST_BITS,
    EVENT_NAMES, CMD_BITS, REG_CMD, REG_TARGET, REG_SORT_CMD,
    read_holding, write_register, bits_str, list_com_ports,
)


# ============================================================================
# 通信层: 后台 Modbus 轮询线程 (对应嵌入式 gw_task 的"生产者"角色)
# ============================================================================
class ModbusThread(threading.Thread):
    """后台串口线程。

    与 UI 通过两个 queue 解耦:
      rx_queue: ("conn", ok, msg) | ("data", regs)
      tx_queue: (reg, value)   —— UI 下发的写请求
    """

    def __init__(self, com: str, baud: int, rx_queue: queue.Queue, tx_queue: queue.Queue):
        super().__init__(daemon=True)
        self.com = com
        self.baud = baud
        self.rx = rx_queue
        self.tx = tx_queue
        self._stop = False
        self.ser = None
        self.connected = False
        self._start_grace = 0.0   # 连接后宽限期(秒), 内不报"无应答"
        self._backoff_ms = 500    # 当前重连退避(ms), 成功即归位, 失败指数退避至 8s

    def run(self):
        while not self._stop:
            # ---- 建连 / 重连 ----
            if not self.connected:
                try:
                    # ★ exclusive=True: Windows 下拒绝其他进程同时 open 同一端口。
                    #   GUI 已占 COM 时, SSCOM/另一个 GUI 实例 open 会立即 PermissionError,
                    #   从根上杜绝"两工具静默串扰抢同一串口"导致的"PC 控制不了"。
                    self.ser = serial.Serial(self.com, self.baud, timeout=0.3,
                                             exclusive=True)
                    self.connected = True
                    self._backoff_ms = 500        # ★ 重连成功 → 退避归位
                    self._start_grace = time.time() + 3.0   # 启动头 3s 静默重试, 不报无应答
                    self.rx.put(("conn", True, f"已连接 {self.com} {self.baud}"))
                except Exception as e:
                    # ★ 指数退避: 500ms → 1s → 2s → 4s → 最大 8s, 不再固定 sleep(2)
                    #   占用方(多为本 GUI 上次未释放/别的串口助手)未关闭前避免忙等空打。
                    self._backoff_ms = min(self._backoff_ms * 2, 8000)
                    self.rx.put(("conn", False,
                        f"打不开 {self.com}: {e} ({self._backoff_ms/1000:.1f}s 后重试)"))
                    time.sleep(self._backoff_ms / 1000)
                    continue

            # ---- 消费 UI 下发指令 ----
            while not self.tx.empty():
                reg, val = self.tx.get()
                try:
                    ok = write_register(self.ser, GW_ADDR, reg, val)
                    if not ok:
                        self.rx.put(("warn", None, f"写 reg{reg} 无应答"))
                except Exception as e:
                    self.rx.put(("warn", None, f"写 reg{reg} 异常: {e}"))

            # ---- 读 15 寄存器 ----
            try:
                regs = read_holding(self.ser, GW_ADDR, 0, GW_REG_COUNT)
            except Exception:
                regs = None

            if regs is None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.connected = False
                # 启动头 3s 宽限期内静默重试, 不报"无应答"(避免上电初始期固件未就绪误报)
                if time.time() < self._start_grace:
                    self.rx.put(("conn", False, "启动中, 稍候重连..."))
                else:
                    self.rx.put(("conn", False, "无应答/超时, 重连..."))
                time.sleep(0.5)   # ★ 原 1s: 1s+0.8s read = 1.8s 空窗, 3s 阈值易被击穿 → PC 误报 DOWN
                continue

            self.rx.put(("data", regs))
            time.sleep(0.5)

    def send(self, reg: int, value: int):
        self.tx.put((reg, value))

    def stop(self):
        self._stop = True
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass


# ============================================================================
# UI 层: 主窗口
# ============================================================================
class Lamp:
    """状态灯小控件: 绿=ON 灰=OFF 红=报警。"""

    COLORS = {"on": "#2ecc71", "off": "#7f8c8d", "alarm": "#e74c3c"}

    def __init__(self, parent, text):
        self.var = tk.StringVar(value=text)
        self.label = tk.Label(parent, textvariable=self.var, width=12,
                              relief="ridge", bd=1, padx=4, pady=2,
                              font=("Microsoft YaHei", 10, "bold"))
        self.label.pack(side="left", padx=3, pady=2)

    def set(self, state: str):
        color = self.COLORS.get(state, self.COLORS["off"])
        self.label.configure(bg=color, fg="#ffffff" if state != "off" else "#000000")


class SorterGUI:
    def __init__(self, root: tk.Tk, default_com: str):
        self.root = root
        self.root.title("双 MCU 物料分拣系统 · 上位机监控")
        cfg = load_config()
        self.root.geometry(cfg.get("geometry", "760x620"))
        self.default_com = cfg.get("com", default_com)

        self.rx = queue.Queue()
        self.tx = queue.Queue()
        self.worker = None
        self.last_event = None
        self.last_regs = None        # 最近一次完整 15 寄存器 (导出用, #51)
        self.sense_hist = deque(maxlen=120)   # (t, sense_a, sense_b) 曲线缓存 (#49)
        self.plot_cnt = 0
        self.cfg_baud = cfg.get("baud", BAUD_DEFAULT)

        self._build_ui()
        self._poll_queue()
        # ★ 关闭窗口时保存配置 (#47)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- 构建界面 ----------
    def _build_ui(self):
        # 连接区
        f = ttk.LabelFrame(self.root, text="连接", padding=6)
        f.pack(fill="x", padx=8, pady=4)
        ttk.Label(f, text="串口:").pack(side="left")
        ports = list_com_ports() or [self.default_com]
        if self.default_com not in ports:
            ports.insert(0, self.default_com)
        # ★ 默认用 self.default_com 而非 ports[0]:
        #   list_com_ports 按系统注册顺序返回, 蓝牙/老设备的虚拟 COM
        #   (如 COM6) 经常排在板载 CH340(COM14) 之前, 默认选 ports[0]
        #   会去打一个无关/被占的端口 → ModbusThread 反复失败 → 期间
        #   固件 3s 心跳超时 → 触发 PC 失联/恢复反复跳。
        self.com_var = tk.StringVar(value=self.default_com)
        self.com_cb = ttk.Combobox(f, textvariable=self.com_var, values=ports,
                                   width=10, state="readonly")
        self.com_cb.pack(side="left", padx=3)
        # ★ 切换串口自动重连: 改端口时若在连接中, 先断开再以新端口连接
        self.com_cb.bind("<<ComboboxSelected>>", self._on_com_changed)
        ttk.Label(f, text="波特:").pack(side="left")
        self.baud_var = tk.StringVar(value=str(self.cfg_baud))
        ttk.Entry(f, textvariable=self.baud_var, width=8).pack(side="left", padx=3)
        self.btn_conn = ttk.Button(f, text="连接", command=self._connect)
        self.btn_conn.pack(side="left", padx=6)
        self.conn_lamp = Lamp(f, "串口连接")
        self.conn_lamp.set("off")

        # 系统状态
        f = ttk.LabelFrame(self.root, text="系统状态", padding=6)
        f.pack(fill="x", padx=8, pady=4)
        self.lamp_online = Lamp(f, "F103在线")
        self.lamp_run = Lamp(f, "运行")
        self.lamp_pcdown = Lamp(f, "PC掉线")
        self.lamp_sort = Lamp(f, "分拣中")
        self.lamp_resend = Lamp(f, "指令积压")        # #33 BIT_RESEND 0x40: 下行 FIFO 非空
        self.sys_var = tk.StringVar(value="SYS_STATUS: --")
        ttk.Label(f, textvariable=self.sys_var).pack(side="left", padx=8)

        # 电机
        f = ttk.LabelFrame(self.root, text="电机", padding=6)
        f.pack(fill="x", padx=8, pady=4)
        ttk.Label(f, text="实际PWM:").pack(side="left")
        self.pwm_var = tk.StringVar(value="0")
        ttk.Label(f, textvariable=self.pwm_var, width=6).pack(side="left")
        self.pwm_bar = ttk.Progressbar(f, maximum=999, length=140)
        self.pwm_bar.pack(side="left", padx=4)
        ttk.Label(f, text="目标:").pack(side="left", padx=(10, 0))
        self.target_var = tk.IntVar(value=0)
        self.target_val_label = ttk.Label(f, text="0", width=4)
        self.target_val_label.pack(side="left")
        # ★ 目标 PWM 滑条 (0..999, 与固件 R_GW_MOTOR_TARGET 一致, #46)
        self.target_scale = ttk.Scale(f, from_=0, to=999, variable=self.target_var, length=130)
        self.target_scale.pack(side="left")
        self.target_scale.configure(
            command=lambda v: self.target_val_label.configure(text=str(int(float(v)))))
        ttk.Button(f, text="下发", command=self._set_target).pack(side="left", padx=3)
        self.motor_var = tk.StringVar(value="状态: -- | 故障: --")
        ttk.Label(f, textvariable=self.motor_var).pack(side="left", padx=8)

        # 传感 + 计数 (一行两列)
        f = ttk.Frame(self.root)
        f.pack(fill="x", padx=8, pady=4)
        fl = ttk.LabelFrame(f, text="传感器 (原始ADC)", padding=6)
        fl.pack(side="left", fill="both", expand=True, padx=2)
        self.sense_a_var = tk.StringVar(value="SENSE_A: --")
        self.sense_b_var = tk.StringVar(value="SENSE_B: --")
        ttk.Label(fl, textvariable=self.sense_a_var).pack(anchor="w")
        ttk.Label(fl, textvariable=self.sense_b_var).pack(anchor="w")

        fr = ttk.LabelFrame(f, text="计数", padding=6)
        fr.pack(side="left", fill="both", expand=True, padx=2)
        self.mat_var = tk.StringVar(value="物料累计: --")
        self.sorta_var = tk.StringVar(value="A料道: --")
        self.sortb_var = tk.StringVar(value="B料道: --")
        ttk.Label(fr, textvariable=self.mat_var).pack(anchor="w")
        ttk.Label(fr, textvariable=self.sorta_var).pack(anchor="w")
        ttk.Label(fr, textvariable=self.sortb_var).pack(anchor="w")

        # SENSE_A/B 历史曲线 (#49): matplotlib 可选, 缺库自动隐藏
        if HAVE_MPL:
            fplot = ttk.LabelFrame(self.root, text="传感器历史曲线 (SENSE_A 蓝 / SENSE_B 绿)", padding=4)
            fplot.pack(fill="x", padx=8, pady=4)
            self.fig = Figure(figsize=(6.4, 1.7), dpi=80)
            self.ax = self.fig.add_subplot(111)
            self.ax.set_ylim(0, 4095)
            self.ax.set_xlabel("秒 (相对)")
            self.line_a, = self.ax.plot([], [], "b-", lw=1.2, label="SENSE_A")
            self.line_b, = self.ax.plot([], [], "g-", lw=1.2, label="SENSE_B")
            self.ax.legend(loc="upper right", fontsize=7)
            self.ax.grid(True, alpha=0.3)
            self.fig.tight_layout()
            self.canvas = FigureCanvasTkAgg(self.fig, master=fplot)
            self.canvas.get_tk_widget().pack(fill="x")
        else:
            self.canvas = None

        # 上电自检 POST
        f = ttk.LabelFrame(self.root, text="上电自检 POST", padding=6)
        f.pack(fill="x", padx=8, pady=4)
        self.lamp_post_adc = Lamp(f, "ADC")
        self.lamp_post_motor = Lamp(f, "电机")
        self.lamp_post_enc = Lamp(f, "编码器")
        self.lamp_post_servo = Lamp(f, "舵机")
        self.post_var = tk.StringVar(value="POST: --")
        ttk.Label(f, textvariable=self.post_var).pack(side="left", padx=8)

        # 黑匣子
        f = ttk.LabelFrame(self.root, text="故障黑匣子 (掉电不丢)", padding=6)
        f.pack(fill="both", expand=True, padx=8, pady=4)
        self.bb_var = tk.StringVar(value="累计 0 条 | 最近: -- | 时间戳 --s")
        ttk.Label(f, textvariable=self.bb_var).pack(anchor="w")
        self.log = scrolledtext.ScrolledText(f, height=7, state="disabled")
        self.log.pack(fill="both", expand=True, pady=3)
        # ★ 日志染色 (#48): INFO 灰 / WARN 黄 / ERROR 红, 时间戳左贴边
        self.log.tag_config("INFO", foreground="#555555")
        self.log.tag_config("WARN", foreground="#b8860b")
        self.log.tag_config("ERROR", foreground="#c0392b")

        # 控制
        f = ttk.LabelFrame(self.root, text="控制指令", padding=6)
        f.pack(fill="x", padx=8, pady=4)
        ttk.Button(f, text="启动", command=lambda: self._send_cmd("启动")).pack(side="left", padx=4)
        ttk.Button(f, text="停止", command=lambda: self._send_cmd("停止")).pack(side="left", padx=4)
        ttk.Button(f, text="复位", command=lambda: self._send_cmd("复位")).pack(side="left", padx=4)
        ttk.Button(f, text="分拣A", command=lambda: self._send_sort(1)).pack(side="left", padx=4)
        ttk.Button(f, text="分拣B", command=lambda: self._send_sort(2)).pack(side="left", padx=4)
        ttk.Button(f, text="舵回中", command=lambda: self._send_sort(0)).pack(side="left", padx=4)
        ttk.Button(f, text="导出", command=self._export).pack(side="left", padx=4)  # #51
        ttk.Label(f, text="(启动/停止/复位写 R_GW_CMD; 分拣写 R_GW_SORT_CMD)").pack(side="left", padx=10)

    # ---------- 连接控制 ----------
    def _on_com_changed(self, event=None):
        """下拉框切换串口: 若正在连接, 自动断开并按新端口重连。"""
        if self.worker and self.worker.is_alive():
            self._disconnect()
        self._connect()

    def _connect(self):
        if self.worker and self.worker.is_alive():
            return
        com = self.com_var.get()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = BAUD_DEFAULT
        self.conn_lamp.set("off")
        self.worker = ModbusThread(com, baud, self.rx, self.tx)
        self.worker.start()
        self.btn_conn.configure(text="断开", command=self._disconnect)

    def _disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.conn_lamp.set("off")
        self.btn_conn.configure(text="连接", command=self._connect)
        self._log_line("已断开")

    # ---------- 队列轮询 (UI 线程, 经 root.after 驱动) ----------
    def _poll_queue(self):
        try:
            while True:
                item = self.rx.get_nowait()
                kind = item[0]
                if kind == "conn":
                    _, ok, msg = item
                    if ok:
                        self.conn_lamp.set("on")
                        self._log_line(msg, "INFO")
                    else:
                        self.conn_lamp.set("off")
                        self._log_line(msg, "WARN")
                elif kind == "warn":
                    self._log_line(item[2], "WARN")
                elif kind == "data":
                    self._update(item[1])
        except queue.Empty:
            pass
        self.root.after(120, self._poll_queue)

    # ---------- 数据刷新 ----------
    def _update(self, regs):
        # 系统状态
        s = regs[7]
        self.lamp_online.set("on" if s & 0x01 else "off")
        self.lamp_run.set("on" if s & 0x02 else "off")
        self.lamp_pcdown.set("alarm" if s & 0x10 else "off")
        self.lamp_sort.set("on" if s & 0x04 else "off")
        self.lamp_resend.set("alarm" if s & 0x40 else "off")   # #33 BIT_RESEND
        self.sys_var.set(f"SYS_STATUS: 0x{s:04X}  {bits_str(s, SYS_STATUS_BITS)}")

        # 电机
        pwm = regs[0]
        self.pwm_var.set(str(pwm))
        self.pwm_bar.configure(value=pwm)
        self.motor_var.set(
            f"状态: {bits_str(regs[2], MOTOR_STATUS_BITS)} | 故障: {bits_str(regs[3], FAULT_BITS)}")

        # 传感 / 计数
        self.sense_a_var.set(f"SENSE_A: {regs[8]}")
        self.sense_b_var.set(f"SENSE_B: {regs[9]}")
        self.mat_var.set(f"物料累计: {regs[4]}")
        self.sorta_var.set(f"A料道: {regs[5]}")
        self.sortb_var.set(f"B料道: {regs[6]}")

        # POST
        p = regs[11]
        self.lamp_post_adc.set("on" if p & 0x01 else "off")
        self.lamp_post_motor.set("on" if p & 0x02 else "off")
        self.lamp_post_enc.set("on" if p & 0x04 else "off")
        self.lamp_post_servo.set("on" if p & 0x08 else "off")
        self.post_var.set(f"POST: 0x{p:04X}  {bits_str(p, POST_BITS)}")

        # 黑匣子
        ev = EVENT_NAMES.get(regs[13], f"未知({regs[13]})")
        self.bb_var.set(f"累计 {regs[12]} 条 | 最近: {ev} | 时间戳 {regs[14]}s")
        if regs[13] != self.last_event:
            self.last_event = regs[13]
            if regs[13] != 0:
                self._log_line(f"事件: {ev}", EVENT_LEVEL.get(regs[13], "INFO"))

        # 保存最近寄存器快照 (导出用, #51)
        self.last_regs = regs

        # SENSE_A/B 历史曲线 (#49): 每 ~1s 重绘一次 (500ms 轮询 × 2)
        self.sense_hist.append((time.time(), regs[8], regs[9]))
        self.plot_cnt += 1
        if HAVE_MPL and self.canvas is not None and self.plot_cnt % 2 == 0:
            self._refresh_plot()

    # ---------- SENSE 历史曲线刷新 (#49) ----------
    def _refresh_plot(self):
        if len(self.sense_hist) < 2:
            return
        base = self.sense_hist[0][0]
        xs = [t - base for t, _, _ in self.sense_hist]
        self.line_a.set_data(xs, [a for _, a, _ in self.sense_hist])
        self.line_b.set_data(xs, [b for _, _, b in self.sense_hist])
        self.ax.relim()
        self.ax.autoscale_view(scalex=True, scaley=False)
        self.canvas.draw_idle()

    # ---------- 导出 CSV + JSON (#51) ----------
    def _export(self):
        if self.last_regs is None:
            messagebox.showinfo("无数据", "还没有读到数据, 请先连接后再导出")
            return
        ts = time.strftime("%Y%m%d_%H%M%S")
        default = Path.home() / f"sorter_dump_{ts}.csv"
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default.name,
            initialdir=str(default.parent),
            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            rows = []
            for i, v in enumerate(self.last_regs):
                name = REG_NAMES[i][0] if i < len(REG_NAMES) else "?"
                rows.append((i, name, v, f"0x{v:04X}"))
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow(["index", "name", "value", "hex"])
                w.writerows(rows)
            jpath = str(Path(path).with_suffix(".json"))
            data = {
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "registers": [
                    {"index": i, "name": n, "value": v, "hex": h} for i, n, v, h in rows
                ],
            }
            with open(jpath, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            self._log_line(f"已导出 {path} + {jpath}", "INFO")
        except Exception as e:
            messagebox.showerror("导出失败", str(e))

    # ---------- 关闭窗口: 保存配置 (#47) ----------
    def _on_close(self):
        try:
            baud = int(self.baud_var.get())
        except Exception:
            baud = BAUD_DEFAULT
        save_config({
            "com": self.com_var.get(),
            "baud": baud,
            "geometry": self.root.geometry(),
        })
        self.root.destroy()

    # ---------- 指令下发 ----------
    def _set_target(self):
        # ★ 目标来自滑条 (#46): IntVar 已 clamp 0..999
        val = int(self.target_var.get())
        if self.worker:
            self.worker.send(REG_TARGET, val)
            self._log_line(f"下发目标 PWM: {val}", "INFO")

    def _send_cmd(self, name):
        bit = CMD_BITS[name]
        if self.worker:
            self.worker.send(REG_CMD, bit)
            self._log_line(f"下发指令: {name}")
        else:
            messagebox.showinfo("未连接", "请先点击连接")

    def _send_sort(self, slot):
        name = "回中" if slot == 0 else ("A料道" if slot == 1 else "B料道")
        if self.worker:
            self.worker.send(REG_SORT_CMD, slot)
            self._log_line(f"下发分拣: {name}")
        else:
            messagebox.showinfo("未连接", "请先点击连接")

    # ---------- 日志 (#48): 统一时间戳 + 级别染色 ----------
    def _log_line(self, text, level="INFO"):
        tag = level if level in ("INFO", "WARN", "ERROR") else "INFO"
        self.log.configure(state="normal")
        self.log.insert("end", f"[{time.strftime('%H:%M:%S')}] ", "INFO")
        self.log.insert("end", f"{text}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")


def main():
    default_com = "COM14"
    for i, a in enumerate(sys.argv):
        if a == "--com" and i + 1 < len(sys.argv):
            default_com = sys.argv[i + 1]
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    SorterGUI(root, default_com)
    root.mainloop()


if __name__ == "__main__":
    main()
