#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>

/* ============================================================================
 * 极简 Modbus RTU 实现（无外部库依赖，便于理解 + 体积可控）
 * 支持功能码：
 *   0x03 读保持寄存器（read holding registers）
 *   0x06 写单个寄存器（write single register）
 *   0x10 写多个寄存器（write multiple registers）
 *
 * 帧格式（RTU，无地址头，靠 3.5 字符静默间隔分帧）：
 *   [ADDR][FC][DATA...][CRC_LO][CRC_HI]
 * ==========================================================================*/

#define MB_FC_READ_HOLDING   0x03
#define MB_FC_WRITE_SINGLE   0x06
#define MB_FC_WRITE_MULTI    0x10

#define MB_MAX_FRAME         256
#define MB_MAX_REGS          125

/* CRC16 (Modbus, poly 0xA001, init 0xFFFF) */
uint16_t mb_crc16(const uint8_t *buf, uint16_t len);

/* 构建请求帧，返回帧长度。out 需 >= MB_MAX_FRAME */
uint16_t mb_build_read_req(uint8_t *out, uint8_t addr, uint16_t reg, uint16_t count);
uint16_t mb_build_write_single_req(uint8_t *out, uint8_t addr, uint16_t reg, uint16_t val);
uint16_t mb_build_write_multi_req(uint8_t *out, uint8_t addr, uint16_t reg,
                                  uint16_t count, const uint16_t *vals);

/* 校验帧（长度>=4 且 CRC 正确返回 1） */
int mb_check_frame(const uint8_t *frame, uint16_t len);

/* 解析读保持寄存器响应，提取寄存器值到 out_regs。
 * 返回提取到的寄存器个数，负数表示错误（见下）。 */
int mb_parse_read_resp(const uint8_t *frame, uint16_t len,
                       uint16_t *out_regs, uint16_t max_regs);
/* 返回值：>0=成功提取的寄存器数；-1=长度不足；-2=从站异常响应；-3=功能码不符 */

/* 从站处理函数：根据请求构建响应。
 * my_addr  : 本机从站地址；请求地址 != my_addr 时不响应（return 0）
 * read_reg  : 读取本地寄存器值（用于 0x03 / 0x06 回显）
 * write_reg : 写入本地寄存器（返回 0 成功，非 0 表示非法地址→返回异常）
 * 返回响应帧长度；返回 0 表示该请求未处理（地址不匹配或不支持的功能码）。
 * resp 需 >= MB_MAX_FRAME。 */
uint16_t mb_slave_handle(const uint8_t *req, uint16_t req_len,
                         uint8_t *resp, uint16_t resp_max,
                         uint8_t my_addr,
                         uint16_t (*read_reg)(uint16_t reg),
                         int (*write_reg)(uint16_t reg, uint16_t val));

#endif /* MODBUS_RTU_H */
