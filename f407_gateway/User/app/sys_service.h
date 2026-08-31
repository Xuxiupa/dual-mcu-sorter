#ifndef SYS_SERVICE_H
#define SYS_SERVICE_H

#include <stdint.h>

/* ================================================================
 * 系统服务层 —— 看门狗任务心跳监督 (F407)
 *
 * 借鉴工业级架构"系统服务层"思想:
 *   - IWDG 喂狗职责从业务任务(GW_Task)下沉到独立 Watchdog_Task;
 *   - 各业务任务每轮仅上报心跳(WD_Task_Alive), 不直接碰 IWDG 寄存器;
 *   - Watchdog_Task 每秒巡检, 任一任务心跳超时(> WD_TIMEOUT_MS)
 *     → 记黑匣子 BB_EV_TASK_TIMEOUT + 停止喂狗
 *     → IWDG(2s) 自然触发整机复位, 实现"系统级"复位语义
 *       (业务任务卡死不再被"单任务喂狗"掩盖)。
 * ================================================================ */

/* 心跳任务 ID (与 freertos.c 中任务一一对应) */
typedef enum {
    WD_TASK_GW = 0,      /* gw_task     高优   20ms  主站轮询/从站应答 */
    WD_TASK_UI,          /* ui_task     低优   200ms OLED 渲染        */
    WD_TASK_ALERT,       /* alert_task  中优   200ms 蜂鸣/LED 报警    */
    WD_TASK_BB,          /* bb_task     最低   1s    黑匣子持久化     */
    WD_TASK_CNT
} wd_task_id_t;

/* 心跳超时阈值(ms): 超过即判该任务卡死。取 5s > BB_Task 1s 节流 + W25Q64
 * 最坏擦写 150ms 余量; 同时小于 IWDG 复位语义可接受范围。 */
#define WD_TIMEOUT_MS  5000U

/* 系统服务层初始化: 心跳数组清零 (APP_MAIN_Init 调用, 调度器启动前) */
void WD_Service_Init(void);

/* 业务任务心跳上报 (各任务主循环每轮调用一次) */
void WD_Task_Alive(wd_task_id_t id);

/* 巡检全部任务心跳。返回: 1=全健康  0=存在超时任务 (Watchdog_Task 每秒调用) */
uint8_t WD_CheckAll(void);

/* ================================================================
 * 看门狗复位事件备份 (BKPSRAM 旁路) — 修复事件7落盘漏洞 (#1)
 *   wdTask 检测到任务超时后 BB_Log(BB_EV_TASK_TIMEOUT) 走队列→W25Q64,
 *   但随后停喂 IWDG, 2s 内复位可能打断 W25Q64 写 → 事件7 落不了盘。
 *   解决: 停喂前把事件码原子写入 BKPSRAM(备份域, IWDG 复位/VBAT 保留, 写<1us
 *   不会被打断), 启动后在稳定上下文(GW_Poll)读出并正式写黑匣子。 */
#define WD_BKP_MAGIC   0xA5U   /* 高8位标记"有效待迁移事件", 区分上电随机值 */
void WD_Bkp_Init(void);                  /* 使能备份域时钟(幂等, 可重复调) */
void WD_Bkp_WriteEvent(uint8_t code);    /* 复位前原子写入待迁移事件码 */
uint8_t WD_Bkp_TakeEvent(uint8_t *code); /* 启动后读出并清零, 返回1=有事件 */

#endif /* SYS_SERVICE_H */
