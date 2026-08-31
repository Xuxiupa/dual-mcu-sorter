#ifndef MODBUS_GATEWAY_H
#define MODBUS_GATEWAY_H

#include <stdint.h>
#include "usart.h"          /* huart1 / huart3 在这里（CubeMX 生成的 .h） */
#include "gpio.h"            /* LED1_Pin / LED1_GPIO_Port 等 GPIO 定义 */
#include "mb_regmap.h"       /* GW_REG_COUNT 等寄存器定义 */

/* F407 网关应用层
 *  - USART3 (huart3, PB10/PB11)：Modbus 主站，轮询 F103 从站(0x01)
 *  - USART1 (huart1, PA9/PA10)：Modbus 从站，应答 PC(地址 0x02)
 *  - 两个 UART 均使用 RXNE 中断 + 环形缓冲（与 F103 已验证方案一致）
 */

void GW_Init(void);
void GW_Poll(void);

/* FreeRTOS 调度器启动后由 gw_task 调用：启动双 UART 的 RXNE 中断接收
 * （队列在 MX_FREERTOS_Init 中已创建，ISR 入队才安全） */
void GW_StartRx(void);

/* gw_task 从字节队列取出 {port,byte} 后喂给对应接收缓冲（RTOS 任务上下文调用） */
void GW_FeedRxByte(uint8_t port, uint8_t ch);

/* UI/按键触发系统指令（启动/停止/复位），转发给 F103 */
void GW_RequestCmd(uint16_t cmd);

/* 由 stm32f1xx_it.c / stm32f4xx_it.c 中的 UART 中断调用 */
void GW_UART_RxISR(UART_HandleTypeDef *huart);

/* 供 PC 从站使用的本地寄存器镜像（网关自身状态 + 从 F103 同步来的数据） */
extern uint16_t g_gw_regs[GW_REG_COUNT];

/* 事件码（经事件队列 q_ev 从 gw_task 发给 alert_task） */
typedef enum {
    GW_EV_NONE = 0,
    GW_EV_F103_UP,       /* F103 上线 */
    GW_EV_F103_DOWN,     /* F103 掉线 */
    GW_EV_FAULT,         /* 出现故障 */
    GW_EV_FAULT_CLEAR,   /* 故障消除 */
    GW_EV_PC_DOWN,       /* PC 心跳超时/上位机失联 */
    GW_EV_PC_UP,         /* PC 心跳恢复 */
} gw_ev_t;

/* 故障黑匣子队列元素 */
typedef struct {
    uint8_t  code;       /* BB_EV_* */
    uint16_t val;
    uint32_t ts;         /* s（HAL_GetTick/1000） */
} bb_qev_t;

/* 故障黑匣子事件码 (BB_Log 的 code 参数) */
#define BB_EV_F103_FAULT   1   /* val = 故障位掩码 */
#define BB_EV_F103_OFFLINE 2
#define BB_EV_FAULT_CLEAR  3
#define BB_EV_PC_DOWN      4
#define BB_EV_PC_UP        5
#define BB_EV_F103_ONLINE  6
#define BB_EV_TASK_TIMEOUT 7   /* 系统服务层: 业务任务心跳超时(整机复位前最后一条) */
#define BB_EV_RESEND_START 8   /* RESYNC 开始: val = 补发队列积压深度 */
#define BB_EV_RESEND_DONE  9   /* RESYNC 完成: 断链期间下行指令已全部重放 */
#define BB_EV_CMD_DROP     10  /* 补发 FIFO 满溢出丢弃: val = 被弃寄存器号 */

/* 下行补发(F407↔F103 断链重同步)专用状态位, 挂在 R_GW_SYS_STATUS(bit6) */
#define BIT_RESEND         0x0040   /* 补发队列非空: PC 可读到"指令积压", 视觉提示 */

/* 记录一条黑匣子事件 (异步: 立即塞队列返回, 持久化由 BB_Task 完成)
 * 供 gw_task / Watchdog_Task 等系统组件调用 */
void BB_Log(uint8_t code, uint16_t val);

/* 黑匣子任务入口（freertos.c BBTask 调它，事件队列消费者） */
void BB_Task_Entry(void *arg);

#endif /* MODBUS_GATEWAY_H */
