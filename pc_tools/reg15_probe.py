#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reg15 写指令诊断: 看 F407 实际回的是什么
  - 8 字节 (02 06 00 0F 00 01 xx xx)  = 正常 ACK, 新固件 case R_GW_SORT_CMD 命中
  - 5 字节 (02 86 02 xx xx)            = Modbus 异常 "非法数据地址" → 老固件 default return -1
  - 0 字节                              = F407 根本没收到 / CRC 错 / 链路问题
用法:  python reg15_probe.py COM14
"""
import sys, time, serial

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

com = sys.argv[1] if len(sys.argv) > 1 else "COM14"
ser = serial.Serial(com, 115200, timeout=0.5)
ser.reset_input_buffer()

# FC 0x06 写 reg15 (0x000F) = 1
req = bytes([0x02, 0x06, 0x00, 0x0F, 0x00, 0x01])
c = crc16(req)
req += bytes([c & 0xFF, c >> 8])

ser.write(req)
time.sleep(0.15)
buf = ser.read(16)

print("TX:", req.hex().upper())
print("RX:", buf.hex().upper() if buf else "(空)")

if not buf:
    print(">> 0 字节: 链路问题/CRC 错/F407 死机")
elif len(buf) == 5 and buf[1] == 0x86:
    print(">> 5 字节异常 0x86 02 = 非法数据地址: 99% 是 F407 没烧新固件, "
          "case R_GW_SORT_CMD 不存在, 落 default return -1")
    print("   修法: Keil Rebuild → F8 Verify OK → 重烧 F407")
elif len(buf) == 8 and buf[1] == 0x06:
    print(">> 8 字节 ACK: F407 新固件工作正常, case R_GW_SORT_CMD 命中, 问题在 PC GUI / 通信层")
else:
    print(">> 异常响应 (非 8 字节 ACK 也非 0x86 异常): 链路干扰, 查接线/CH340/波特")

ser.close()
