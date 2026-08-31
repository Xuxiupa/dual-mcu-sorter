#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F407 网关实时监视器 (命令行版上位机)  —— 双 MCU 物料分拣系统
=============================================================================
读取 F407 Modbus 从站(addr=0x02) 的 15 个保持寄存器, 翻译为可读状态, 自动重连。

用法:
  python live_view.py --com COM14 [--baud 115200] [--poll 1.0]
  python live_view.py --demo            # 用假数据演示翻译效果(不连串口)

依赖: pip install pyserial
寄存器/位定义与 F407/User/app/mb_regmap.h 严格同步。
"""
import sys
import time
import argparse

try:
    import serial
except ImportError:
    sys.exit("[ERROR] 需要 pyserial, 请先: pip install pyserial")

GW_ADDR = 0x02
GW_REG_COUNT = 15

# ---- 寄存器名 (与 mb_regmap.h 对齐) ----
REG_NAMES = [
    "MOTOR_PWM     (镜像F103实际PWM)",
    "MOTOR_TARGET  (PC下发目标0..999)",
    "MOTOR_STATUS  (镜像F103状态)",
    "MOTOR_FAULT   (镜像F103故障码)",
    "MAT_CNT       (物料累计)",
    "SORT_A_CNT    (A料道计数)",
    "SORT_B_CNT    (B料道计数)",
    "SYS_STATUS    (系统状态)",
    "SENSE_A       (传感器A)",
    "SENSE_B       (传感器B)",
    "CMD           (系统指令)",
    "POST          (上电自检)",
    "LOG_CNT       (黑匣子事件数)",
    "LOG_LAST      (最近事件码)",
    "LOG_TS        (最近事件时间戳s)",
]

# ---- 状态位定义 ----
SYS_STATUS_BITS = [
    (0x01, "F103在线"),
    (0x02, "运行"),
    (0x10, "PC掉线(BIT_PC_DOWN)"),
    (0x20, "PC已连过(BIT_PC_SEEN)"),
]
MOTOR_STATUS_BITS = [
    (0x01, "运行(RUN)"),
    (0x02, "故障(FAULT)"),
    (0x04, "分拣中(SORTING)"),
    (0x08, "网关掉线(GW_DOWN)"),
]
FAULT_BITS = [
    (0x01, "失速STALL"),
    (0x02, "超温OVERHEAT"),
    (0x04, "堵料BLOCKED"),
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
# 黑匣子事件码 (与 modbus_gateway.h 的 BB_EV_* 对齐, 由 R_GW_LOG_LAST 上报)
EVENT_NAMES = {
    0:  "NONE",
    1:  "FAULT(故障)",
    2:  "F103_DOWN(掉线)",
    3:  "FAULT_CLEAR(故障消除)",
    4:  "PC_DOWN(PC失联)",
    5:  "PC_UP(PC恢复)",
    6:  "F103_UP(上线)",
    7:  "TASK_TIMEOUT(任务超时)",
    8:  "RESYNC(补发开始)",
    9:  "RESEND_DONE(补发完成)",
    10: "CMD_DROP(FIFO满丢弃)",
    7: "TASK_TIMEOUT(任务超时)",
}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def read_holding(ser, addr: int, reg: int, count: int, timeout: float = 0.4):
    """读保持寄存器, 返回 list[int] 或 None(超时/CRC错)。"""
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
    if crc16(bytes(buf[:expected])) & 0xFFFF != 0:
        return None
    regs = []
    for i in range(count):
        hi = buf[3 + i * 2]
        lo = buf[4 + i * 2]
        regs.append((hi << 8) | lo)
    return regs


def bits_str(value: int, bitdefs) -> str:
    parts = [name for mask, name in bitdefs if value & mask]
    return " | ".join(parts) if parts else "无"


def render(regs, com: str, baud: int) -> str:
    now = time.strftime("%H:%M:%S")
    L = []
    L.append(f"=== F407 网关实时监视  @ {now}   COM={com} {baud} 8N1 ===")
    L.append("")
    for i, v in enumerate(regs):
        L.append(f"  [{i:2d}] {v:6d}  0x{v:04X}  {REG_NAMES[i]}")
    L.append("")
    L.append(f"  SYS_STATUS  = 0x{regs[7]:04X}   {bits_str(regs[7], SYS_STATUS_BITS)}")
    L.append(f"  MOTOR_STATUS= 0x{regs[2]:04X}   {bits_str(regs[2], MOTOR_STATUS_BITS)}")
    L.append(f"  FAULT       = 0x{regs[3]:04X}   {bits_str(regs[3], FAULT_BITS)}")
    L.append(f"  POST        = 0x{regs[11]:04X}   {bits_str(regs[11], POST_BITS)}")
    L.append("")
    ev = EVENT_NAMES.get(regs[13], f"未知({regs[13]})")
    L.append(f"  黑匣子: 累计 {regs[12]} 条 | 最近事件 = {ev} | 时间戳 {regs[14]}s")
    L.append("")
    L.append("  [Ctrl+C 退出]   黑匣子仅镜像最近 1 条(固件当前设计)")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="F407 网关命令行监视器")
    ap.add_argument("--com", default="COM14", help="F407 板载 CH340G 的 COM 口")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--poll", type=float, default=1.0, help="刷新间隔(秒)")
    ap.add_argument("--demo", action="store_true", help="用假数据演示(不连串口)")
    args = ap.parse_args()

    if args.demo:
        # 演示: 电机运行中+网关掉线, 故障=失速+光敏断线, PC已连, POST全过, 黑匣子177条/最近PC失联
        demo = [250, 250, 0x0009, 0x0018, 12, 3, 5, 0x0031, 1825, 1826,
                0, 0x0007, 177, 4, 328]
        print(render(demo, args.com, args.baud))
        return

    ser = None
    while True:
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(args.com, args.baud, timeout=0.2)
            except Exception as e:
                sys.stdout.write(f"\r[等待] 打不开 {args.com}: {e}  (2s 重试)")
                sys.stdout.flush()
                time.sleep(2)
                continue
        regs = read_holding(ser, GW_ADDR, 0, GW_REG_COUNT)
        if regs is None:
            sys.stdout.write("\r[--] 无应答(超时/CRC错), 重连... ")
            sys.stdout.flush()
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(1)
            continue
        sys.stdout.write("\033[2J\033[H")  # 清屏
        sys.stdout.write(render(regs, args.com, args.baud) + "\n")
        sys.stdout.flush()
        time.sleep(args.poll)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[退出]")
