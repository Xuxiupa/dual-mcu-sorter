/* ================================================================
 * 系统服务层 —— 看门狗任务心跳监督 (F407)
 *   见 sys_service.h 头注释。纯软件实现, 不依赖 CubeMX 外设配置。
 * ================================================================ */

#include "sys_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"   /* PWR / BKPSRAM 宏 + HAL_PWR_EnableBkUpAccess */

/* 各任务最近一次心跳 tick (WD_Task_Alive 写, Watchdog_Task 巡检读) */
static volatile uint32_t s_alive_ms[WD_TASK_CNT];

void WD_Service_Init(void)
{
    uint8_t i;
    for (i = 0; i < WD_TASK_CNT; i++) s_alive_ms[i] = 0;
    WD_Bkp_Init();   /* 使能备份域时钟, 复位后 BKPSRAM 内容(看门狗事件)可读取 */
}

/* 备份域(BKPSRAM)访问使能: 幂等, 任意时刻调用都安全。
 * BKPSRAM 在 V_BAT 域, IWDG 复位(非掉电)内容保留, 但 RCC 时钟位会随复位清零,
 * 故每次访问前都重新使能 PWR 备份访问 + BKPSRAM 时钟。 */
static void bkp_enable(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
}

void WD_Bkp_Init(void) { bkp_enable(); }

/* 原子写入待迁移事件码到 BKPSRAM 首 4 字节(高8位 magic 标记有效, 次高8位存事件码)。
 * BKPSRAM 写入是单条 SRAM 总线操作(<1us), 不会被随后 0~2s 的 IWDG 复位打断。 */
void WD_Bkp_WriteEvent(uint8_t code)
{
    bkp_enable();
    *(__IO uint32_t *)BKPSRAM_BASE =
        (uint32_t)((WD_BKP_MAGIC << 24U) | ((uint32_t)code << 16U));
}

/* 启动后读出待迁移事件码并清零(防重复迁移)。返回 1=有有效事件, 0=无。 */
uint8_t WD_Bkp_TakeEvent(uint8_t *code)
{
    bkp_enable();
    uint32_t v = *(__IO uint32_t *)BKPSRAM_BASE;
    if ((v >> 24U) != WD_BKP_MAGIC) { *code = 0; return 0; }
    *code = (uint8_t)((v >> 16U) & 0xFFU);
    *(__IO uint32_t *)BKPSRAM_BASE = 0;   /* 清掉, 防重复迁移 */
    return 1;
}

void WD_Task_Alive(wd_task_id_t id)
{
    if (id >= WD_TASK_CNT) return;
    s_alive_ms[id] = (uint32_t)xTaskGetTickCount();
}

uint8_t WD_CheckAll(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint8_t  i;
    for (i = 0; i < WD_TASK_CNT; i++)
    {
        /* 32bit 减法: tick 回绕也安全 */
        if ((uint32_t)(now - s_alive_ms[i]) > WD_TIMEOUT_MS) return 0;
    }
    return 1;
}
