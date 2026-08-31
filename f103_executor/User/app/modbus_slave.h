#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include <stdint.h>

/* ============================================================================
 * F103 端 Modbus RTU 从站（地址 0x01）
 *
 * 公共层（复用 F407 验证版本）：
 *   - User/middleware/ringbuf.h
 *   - User/middleware/modbus_rtu.h  (带 my_addr 入参版本)
 *   - User/middleware/mb_regmap.h  (F103/F407 共享寄存器映射)
 *
 * USART 占用（与 F407 的串口分配对仗）：
 *   USART1 = 调试串口 (printf + SSCOM 命令解析) — 原有 app_main.c 维护
 *   USART2 = Modbus RTU 从站                  — 本文件维护
 *
 * 关键设计：USART1 + USART2 的 RXNE 中断入口 (HAL_UART_RxCpltCallback)
 * 只能定义一次，由本文件统一接管。USART1 的字节通过 APP_UART1_RxByte()
 * 转交给 app_main.c 的原环形缓冲，命令解析逻辑一行不动。
 * ==========================================================================*/

/* 初始化：启动 USART1 + USART2 RXNE 中断接收。
 * 调用前必须保证 MX_USART1_UART_Init() 和 MX_USART2_UART_Init() 已执行
 * （即 huart1 / huart2 已非 NULL）。 */
void MODBUS_SLAVE_Init(void);

/* 主循环轮询：
 *   1) 处理 USART2 接收缓冲 — 5ms 静默分帧，调 mb_slave_handle 拼响应。
 *   2) 把 PC 下发的目标速度 / 控制位同步到 F103 应用层（电机/状态机）。
 *   3) 把 F103 应用层当前状态写回寄存器镜像（供 PC 读）。
 *
 * 必须每 ~10ms 调用一次。 */
void MODBUS_SLAVE_Poll(void);

/* USART1 接收字节入队 — 由 modbus_slave.c 的 HAL_UART_RxCpltCallback 调用。
 * app_main.c 的原 USART1 串口命令解析逻辑完全保留，只是字节从这里过来。 */
void APP_UART1_RxByte(uint8_t b);

/* 物料到达通知 — 在 PA12 EXTI 下降沿钩子(HAL_GPIO_EXTI_Callback)里调用。
 * 累加 R_F103_MAT_CNT (reg 4)。 */
void MODBUS_SLAVE_NotifyMatArrived(void);

/* POST 上电自检结果上报 — app_main.c 开机跑完 post_run() 后调用一次,
 * 把结果位掩码传给从站, 由 R_F103_POST(reg9) 经 Modbus 上送 F407/PC。 */
void MODBUS_SLAVE_ReportPost(uint16_t mask);

/* 分拣完成通知 — 1=A 料道, 2=B 料道。
 * 累加 R_F103_SORT_A_CNT (reg 5) 或 R_F103_SORT_B_CNT (reg 6)。
 * 由舵机动作回调或主循环根据 SORT_CMD 完成时调用, 先空调用即可。 */
void MODBUS_SLAVE_NotifySortDone(uint8_t slot);

/* USART1 接收字节出队 — app_main.c 主循环调用，返回 1=成功取到一个字节，0=空。
 * 用法:
 *     uint8_t ch;
 *     while (APP_UART1_GetByte(&ch)) { ... 处理 ch ... }
 * 配合"超时触发 parse_line"逻辑，与原 app_main.c 一致。 */
int APP_UART1_GetByte(uint8_t *b);

/* 分拣/复检阈值运行期标定接口 (#5): 串口命令 TL/TR/CL 经 app_main.c 调用,
 * 实际变量在 modbus_slave.c 内(static), 经访问器读写避免跨文件暴露内部状态。 */
void    MODBUS_SLAVE_SetLightThresh(uint16_t v);
void    MODBUS_SLAVE_SetRecheckThresh(uint16_t v);
uint16_t MODBUS_SLAVE_GetLightThresh(void);
uint16_t MODBUS_SLAVE_GetRecheckThresh(void);

#endif /* __MODBUS_SLAVE_H */
