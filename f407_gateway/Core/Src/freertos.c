/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_gateway.h"   /* GW_Poll / GW_StartRx / GW_FeedRxByte / gw_ev_t */
#include "ui.h"               /* UI_Refresh */
#include "gpio.h"             /* GPIOF / GPIO_PIN_9 (LED0) */
#include "iwdg.h"             /* hiwdg + HAL_IWDG_Refresh (阶段10 看门狗) */
#include "sys_service.h"      /* WD_Task_Alive / WD_CheckAll (系统服务层看门狗) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t g_q_rx;   /* 字节队列: 元素 uint16_t=(port<<8)|byte, ISR→gw_task */
osMessageQueueId_t g_q_ev;   /* 事件队列: 元素 uint8_t=gw_ev_t, gw_task→alert_task */
osMessageQueueId_t g_q_bb;   /* 黑匣子队列: 元素 bb_qev_t, gw_task→bb_task (异步 W25Q64 写) */

/* 任务属性: gw 最高(轮询+应答不能被 UI 拖), alert 次之, ui 较低(慢 OLED 不抢时间),
 * bb 最低(W25Q64 擦扇区 100ms,即使阻塞也不卡 PC 链路) */
const osThreadAttr_t gwTask_attributes = {
  .name = "gwTask", .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
const osThreadAttr_t uiTask_attributes = {
  .name = "uiTask", .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
const osThreadAttr_t alertTask_attributes = {
  .name = "alertTask", .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* 黑匣子任务: 优先级 osPriorityLow, 低于 UI, 持久化阻塞不被感知 */
const osThreadAttr_t bbTask_attributes = {
  .name = "bbTask", .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* 系统服务层看门狗任务: 优先级 osPriorityIdle(最低), 每秒巡检各业务任务心跳
 * 并独占 IWDG 喂狗。纯 osDelay 无阻塞; 若自身卡死, IWDG 兜底复位。 */
const osThreadAttr_t wdTask_attributes = {
  .name = "wdTask", .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityIdle,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void GW_Task(void *argument);
void UI_Task(void *argument);
void Alert_Task(void *argument);
void BB_Task_Entry(void *argument);   /* 故障黑匣子: 异步持久化到 W25Q64 */
void WD_Task_Entry(void *argument);   /* 系统服务层看门狗: 巡检心跳 + 独占喂狗 */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  g_q_rx = osMessageQueueNew(64, sizeof(uint16_t), NULL);
  g_q_ev = osMessageQueueNew(8,  sizeof(uint8_t),  NULL);
  g_q_bb = osMessageQueueNew(32, sizeof(bb_qev_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadNew(GW_Task,       NULL, &gwTask_attributes);
  osThreadNew(UI_Task,       NULL, &uiTask_attributes);
  osThreadNew(Alert_Task,    NULL, &alertTask_attributes);
  osThreadNew(BB_Task_Entry, NULL, &bbTask_attributes);
  osThreadNew(WD_Task_Entry, NULL, &wdTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ================================================================
 * gw_task —— 网关核心（高优先级）
 *  1) 从字节队列取 {port,byte}（ISR 用 PutFromISR 入队）喂给接收缓冲
 *  2) GW_Poll(): 主站轮询 F103 + 从站应答 PC + 数据镜像 + LED + 事件入队
 *  每 20ms 周期执行；队列为空时 osMessageQueueGet 立即返回(超时0)不阻塞。
 * ================================================================ */
void GW_Task(void *argument)
{
    GW_StartRx();   /* 调度器已就绪、队列已创建，此刻武装 RXNE 才安全 */

    for (;;)
    {
        //osDelay(15000);   /* ★ 临时验证看门狗: 故意卡死 gw_task, 验证后必须删除本行 */
        WD_Task_Alive(WD_TASK_GW);   /* 心跳上报 (系统服务层), IWDG 喂狗已下沉 wd_task */
        uint16_t item;
        while (osMessageQueueGet(g_q_rx, &item, NULL, 0) == osOK)
        {
            GW_FeedRxByte((uint8_t)(item >> 8), (uint8_t)(item & 0xFF));
        }
        GW_Poll();
        osDelay(20);
    }
}

/* ================================================================
 * ui_task —— OLED 渲染 + KEY 扫描（低优先级，慢 I2C 不抢 gw 的时间）
 * ================================================================ */
void UI_Task(void *argument)
{
    for (;;)
    {
        WD_Task_Alive(WD_TASK_UI);   /* 心跳上报 (系统服务层) */
        UI_Refresh();   /* 内部已有 200ms 节流 */
        osDelay(200);
    }
}

/* ================================================================
 * alert_task —— 告警（中优先级）
 *  事件驱动 + 实时状态兜底:
 *   - PC 心跳: 仅事件驱动 (沿变有效)
 *   - F103 / FAULT: 200ms 重读 g_gw_regs,按最新状态判定,
 *                  防止事件队列塞满/丢失导致永报警
 *  PC 失联 / F103 不在线 / 有 Fault → 蜂鸣 0.5s 周期 + LED0(PF9) 闪;
 *  全部正常 → 静音 + LED0 常亮(链路正常指示)。
 * ================================================================ */
void Alert_Task(void *argument)
{
    uint8_t ev;
    uint8_t s_alarm_pc   = 0;   /* PC 失联  (仅事件驱动) */
    uint8_t s_alarm_sys  = 0;   /* F103/FAULT 综合 (实时校准) */
    uint32_t last_check  = 0;

    BEEP_Init();   /* 启动 TIM13 PWM (初始静音) */

    for (;;)
    {
        WD_Task_Alive(WD_TASK_ALERT);   /* 心跳上报 (系统服务层) */

        /* 取事件 (100ms 超时, 不阻塞 → 报警态可持续刷新) */
        if (osMessageQueueGet(g_q_ev, &ev, NULL, 100) == osOK)
        {
            if (ev == GW_EV_PC_DOWN) s_alarm_pc = 1;
            if (ev == GW_EV_PC_UP)   s_alarm_pc = 0;
            /* F103/FAULT 事件不再累加, 完全由实时状态决定 */
        }

        /* 每 200ms 重读 g_gw_regs 校准 F103 / Fault 报警:
         *   - 实时, 事件丢失也无所谓
         *   - F103 在线 (BIT_LINK on & !BIT_GW_DOWN) → 静
         *   - FAULT==0 → 静 */
        uint32_t now = HAL_GetTick();
        if ((now - last_check) >= 200U)
        {
            last_check = now;
            uint16_t sys   = g_gw_regs[R_GW_SYS_STATUS];
            uint16_t fault = g_gw_regs[R_GW_MOTOR_FAULT];
            uint8_t f103_ok = (sys & BIT_LINK) && !(sys & BIT_GW_DOWN);
            s_alarm_sys = (f103_ok && fault == FAULT_NONE) ? 0 : 1;
        }

        if (s_alarm_pc || s_alarm_sys)
        {
            /* 报警态: 蜂鸣 0.5s 周期 + LED0(PF9) 闪烁 (RESET=亮) */
            BEEP_Set(1);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
            osDelay(250);
            BEEP_Set(0);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
            osDelay(250);
        }
        else
        {
            /* 正常: 静音 + LED0 常亮 */
            BEEP_Set(0);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
            osDelay(100);
        }
    }
}

/* ================================================================
 * wd_task —— 系统服务层看门狗（最低优先级 osPriorityIdle）
 *   借鉴工业架构"系统服务层": IWDG 喂狗职责从 GW_Task 下沉至此,
 *   各业务任务仅上报心跳(WD_Task_Alive), 由本任务统一巡检。
 *   - 每秒巡检 4 个业务任务心跳:
 *       全健康  → HAL_IWDG_Refresh(正常喂狗)
 *       有超时  → 记黑匣子 BB_EV_TASK_TIMEOUT + 停止喂狗
 *                → IWDG(2s) 自然触发整机复位, 实现"系统级"复位语义
 *                  (单个业务任务卡死不再被"单任务喂狗"掩盖)
 *   - 本任务纯 osDelay 无阻塞; 若自身卡死, IWDG 兜底复位(自我监督)
 * ================================================================ */
void WD_Task_Entry(void *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(1000);          /* 每秒巡检一次 */
        if (WD_CheckAll())
        {
            HAL_IWDG_Refresh(&hiwdg);   /* 全部任务健康 → 正常喂狗 */
        }
        else
        {
            /* 有任务超时: 先把事件码原子写入备份域(BKPSRAM, 复位不丢), 再停喂
               → ~2s 后 IWDG 复位整机。BKPSRAM 写 <1us 不会被 IWDG 打断,
               而 BB_Log 走队列→W25Q64 写可能被复位打断 → 事件7落不了盘。
               复位后由 GW_Poll 把备份域事件迁移进黑匣子(#1)。 */
            WD_Bkp_WriteEvent(BB_EV_TASK_TIMEOUT);
            BB_Log(BB_EV_TASK_TIMEOUT, 0);
        }
    }
}

/* USER CODE END Application */

