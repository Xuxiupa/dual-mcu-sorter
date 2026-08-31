#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双 MCU 物料分拣系统 - 上位机寄存器/位定义 (与固件 mb_regmap.h 严格同步)
===========================================================================
纯数据 + 纯函数, 不依赖 UI / 串口库, 可单独单测。

GUI(sorter_gui.py) 与命令行监视器(live_view.py) 共用本模块,
保证"翻译逻辑唯一来源" —— 改固件寄存器语义时只改这里一处。

架构角色: 数据层 (Data Layer)
  与嵌入式端的 modbus_gateway.h / mb_regmap.h 对应, 是 PC 侧对固件
  寄存器语义的唯一真源。 通信层/UI 层都不应各自硬编码位定义。
"""
import time
try:
    import serial
except ImportError:
    raise SystemExit("缺少 pyserial, 请先安装: pip install pyserial")

GW_ADDR = 0x02          # F407 网关 Modbus 从站地址
GW_REG_COUNT = 16       # 从站暴露的保持寄存器总数
BAUD_DEFAULT = 115200

# ---- 寄存器索引 → (名字, 说明) ----
# 顺序严格对齐 mb_regmap.h 的 R_GW_* 定义
REG_NAMES = [
    ("MOTOR_PWM",    "镜像 F103 实际 PWM"),
    ("MOTOR_TARGET", "PC 下发目标 0..999 (W)"),
    ("MOTOR_STATUS", "镜像 F103 状态"),
    ("MOTOR_FAULT",  "镜像 F103 故障码"),
    ("MAT_CNT",      "物料累计计数"),
    ("SORT_A_CNT",   "A 料道分拣计数"),
    ("SORT_B_CNT",   "B 料道分拣计数"),
    ("SYS_STATUS",   "系统状态"),
    ("SENSE_A",      "传感器 A (反射/光敏)"),
    ("SENSE_B",      "传感器 B (热敏)"),
    ("CMD",          "系统指令 (W)"),
    ("POST",         "上电自检结果"),
    ("LOG_CNT",      "黑匣子累计事件数"),
    ("LOG_LAST",     "最近事件码"),
    ("LOG_TS",       "最近事件时间戳 (秒)"),
    ("SORT_CMD",     "分拣指令 (W) 0无 1A 2B"),
]

# ---- 状态位定义 ----
SYS_STATUS_BITS = [
    (0x01, "F103在线"),
    (0x02, "运行"),
    (0x10, "PC掉线"),
    (0x20, "PC已连过"),
    (0x40, "指令积压"),         # BIT_RESEND #33: 下行 FIFO 非空 (重连后逐条补发)
]
MOTOR_STATUS_BITS = [
    (0x01, "运行(RUN)"),
    (0x02, "故障(FAULT)"),
    (0x04, "分拣中(SORTING)"),
    (0x08, "网关掉线(GW_DOWN)"),
    (0x10, "复检通过"),          # BIT_RECHECK_OK  PB1 反射红外确认物料在分拣位
    (0x20, "复检失败"),          # BIT_RECHECK_FAIL PB1 复检未通过(疑似 PA12 误触发)
]
FAULT_BITS = [
    (0x01, "失速"),
    (0x02, "超温"),
    (0x04, "堵料"),
    (0x08, "热敏断线"),
    (0x10, "光敏断线"),
    (0x20, "反射断线"),
    (0x40, "POST未过"),
]
POST_BITS = [
    (0x01, "ADC"),
    (0x02, "电机"),
    (0x04, "编码器"),
    (0x08, "舵机"),
]

# ---- 黑匣子事件码 (与 modbus_gateway.h 的 BB_EV_* 对齐, 由 R_GW_LOG_LAST 上报) ----
EVENT_NAMES = {
    0:  "NONE",
    1:  "FAULT(故障)",
    2:  "F103掉线",
    3:  "故障消除",
    4:  "PC失联",
    5:  "PC恢复",
    6:  "F103上线",
    7:  "任务超时(看门狗)",
    8:  "补发开始(RESYNC)",   # #33: F103 重新上线后开始重放下行 FIFO
    9:  "补发完成",          # #33: FIFO 清空, 链路恢复
    10: "指令丢弃(FIFO满)",  # #33: 8 槽 FIFO 溢出, 最早一条 cmd 被丢弃
}

# ---- 系统指令位 (写 R_GW_CMD / R_F103_CTRL) ----
CMD_BITS = {"启动": 0x0001, "停止": 0x0002, "复位": 0x0004}

# ---- 寄存器写地址 (供 UI 下发) ----
REG_CMD = 10          # R_GW_CMD
REG_TARGET = 1        # R_GW_MOTOR_TARGET
REG_SORT_CMD = 15     # R_GW_SORT_CMD  (PC 远程触发分拣)


def crc16(data: bytes) -> int:
    """Modbus RTU CRC-16 (多项式 0xA001, 初值 0xFFFF)。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def read_holding(ser, addr: int, reg: int, count: int, timeout: float = 0.8):
    """读保持寄存器 FC 0x03, 返回 list[int] 或 None (超时/CRC 错)。
    timeout 默认 0.8s (原 0.4s): F407 主站轮询 F103 + 镜像同步 + 黑匣子写可能 stall,
    偶发 200-400ms 响应, 0.4s 易误判超时 → ModbusThread 进重连 → 期间 USART1 静默 →
    F407 端 s_usart1_last_tick 不刷新 → 3s 心跳超时 → 误报 PC_DOWN → 反复跳。"""
    frame = bytes([addr, 0x03, reg >> 8, reg & 0xFF, count >> 8, count & 0xFF])
    crc = crc16(frame)
    req = frame + bytes([crc & 0xFF, crc >> 8])
    ser.reset_input_buffer()
    ser.write(req)
    buf = bytearray()
    t0 = time.time()
    expected = 5 + count * 2
    while time.time() - t0 < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if len(buf) >= expected:
                break
        time.sleep(0.005)
    if len(buf) < expected:
        return None
    if crc16(bytes(buf[:expected])) & 0xFFFF:
        return None
    return [(buf[3 + i * 2] << 8) | buf[4 + i * 2] for i in range(count)]


def write_register(ser, addr: int, reg: int, value: int, timeout: float = 0.4) -> bool:
    """写单个保持寄存器 FC 0x06, 返回 True/False。"""
    frame = bytes([addr, 0x06, reg >> 8, reg & 0xFF, value >> 8, value & 0xFF])
    crc = crc16(frame)
    req = frame + bytes([crc & 0xFF, crc >> 8])
    ser.reset_input_buffer()
    ser.write(req)
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if len(buf) >= 8:
                break
        time.sleep(0.005)
    if len(buf) < 8:
        return False
    return crc16(bytes(buf[:8])) == 0


def bits_str(value: int, bitdefs) -> str:
    """把位掩码翻译成可读字符串, 未置位返回 '无'。"""
    parts = [name for mask, name in bitdefs if value & mask]
    return " | ".join(parts) if parts else "无"


def list_com_ports():
    """列出当前可用串口 (用于 UI 下拉框), 失败返回空列表。"""
    try:
        from serial.tools.list_ports import comports
        return [p.device for p in comports()]
    except Exception:
        return []
