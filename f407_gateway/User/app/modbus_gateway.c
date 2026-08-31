#include "modbus_gateway.h"
#include "modbus_rtu.h"
#include "mb_regmap.h"
#include "ringbuf.h"
#include "cmsis_os.h"       /* 任务上下文用 osMessageQueuePut/Get */
#include "queue.h"          /* ISR 用 xQueueSendFromISR (CMSIS_V2 适配没实现 PutFromISR) */

#include <string.h>
#include "bsp_w25q64.h"     /* W25Q64_SaveSafe/LoadSafe (故障黑匣子持久化) */
#include "sys_service.h"    /* WD_Task_Alive (BB_Task 心跳上报) */

/* FreeRTOS 队列（定义在 freertos.c 的 USER CODE 块，MX_FREERTOS_Init 创建）：
 *   g_q_rx : 字节队列, 元素 uint16_t = (port<<8)|byte, ISR→gw_task
 *   g_q_ev : 事件队列, 元素 uint8_t  = gw_ev_t,        gw_task→alert_task
 *   g_q_bb : 黑匣子队列, 元素 bb_qev_t,                gw_task→bb_task */
extern osMessageQueueId_t g_q_rx;
extern osMessageQueueId_t g_q_ev;
extern osMessageQueueId_t g_q_bb;

/* ================================================================
 * 板载 LED (LED0=PF9 / LED1=PF10) — 低电平亮(active-low)
 *   GPIO 配置 (GPIO_Output PP) 由 CubeMX 生成的 MX_GPIO_Init() 统一完成,
 *   本文件不手写 HAL_GPIO_Init / __HAL_RCC_GPIOF_CLK_ENABLE。
 *   LED1(PF10) 作 F103 在线指示：在线写 RESET(亮)，掉线写 SET(灭)。
 * ================================================================ */

/* ====================== 私有状态 ====================== */
static ringbuf_t s_rb1;   /* USART1 (PC 从站) 接收缓冲 */
static ringbuf_t s_rb3;   /* USART3 (F103 主站) 接收缓冲 */

static uint32_t s_usart1_last_tick = 0;
static uint32_t s_usart3_last_tick = 0;
static uint8_t  s_pc_seen = 0;    /* PC 至少连过一次: 区分"上电未连"与"连上后掉线" */
static uint8_t  s_pc_down_pending = 0xFF;  /* PC 失联 1s 稳定确认 (避 pyserial 阻塞翻转刷屏) */
static uint32_t s_pc_change_tick   = 0;    /* 上次状态翻转的时间戳 */
#define PC_HEARTBEAT_MS   12000   /* PC 超过 12s 未发任何请求 → 判上位机失联
                                    *   留足 9s 余量, 容忍 Windows USB 转串口驱动
                                    *   (CH340) 偶发 ser.write 阻塞 3-8s+:
                                    *   GUI 每 500ms 发一次读, write 阻塞期间
                                    *   USART1 无字节 → s_usart1_last_tick 不刷 →
                                    *   原 3s 阈值会误判 PC_DOWN。先后试 3→8→12s
                                    *   (2026-08-27): 8s 仍偶发击穿, 12s 留 4s 余量
                                    *   应对 Windows 偶发最长 8s+ 阻塞。
                                    *   权衡: PC 真失联 12s 才报警 (本项目非安全关键,
                                    *   仅 UI 提示, 可接受)。*/

uint16_t g_gw_regs[GW_REG_COUNT];   /* PC 可见的网关寄存器镜像 */

/* 主站轮询状态 */
static uint32_t s_last_poll = 0;
#define MASTER_PERIOD_MS   200      /* 每 200ms 轮询一次 F103 */
#define MASTER_TIMEOUT_MS  120      /* 单次事务等待响应超时 */

/* 待转发给 F103 的写请求（由 PC 写入触发） */
static uint8_t  s_pending_target = 0;      /* COALESCE: 目标PWM 合并槽(只留最新) */
static uint16_t s_pending_target_val = 0;
/* DELIVER-ALL 类(CMD/SORT_CMD)走 s_cmd_fifo 逐条补发; COALESCE 类(目标PWM)用合并槽只留最新 */

static uint8_t  s_f103_online = 0;  /* F103 链路状态 */
static uint8_t  s_offline_cnt = 0;  /* 连续读失败计数，用于去抖判掉线 */

/* ---- 下行指令补发 FIFO (F407↔F103 断链重同步) ----
 * DELIVER-ALL 类(控制指令 R_GW_CMD→R_F103_CTRL、分拣 R_GW_SORT_CMD→R_F103_SORT_CMD)
 *   逐条必达: 在线时立即发, 失败留队; 断链期间 PC 下发则积压, F103 重新上线触发
 *   RESYNC 逐条重放。
 * COALESCE 类(目标PWM)不入队, 用下方合并槽只留最新值。 */
#define CMD_FIFO_DEPTH  8
typedef struct { uint16_t reg; uint16_t val; } cmd_fifo_ent_t;
static cmd_fifo_ent_t s_cmd_fifo[CMD_FIFO_DEPTH];
static uint8_t  s_fifo_head = 0;   /* 出队位置 */
static uint8_t  s_fifo_tail = 0;   /* 入队位置 */
static uint8_t  s_fifo_cnt  = 0;
static uint8_t  s_resync_pending = 0;  /* RESYNC 进行中(上线沿变置位) */

/* 入队一条 DELIVER-ALL 指令; 满则丢最旧 + 记黑匣子 */
static void cmd_fifo_push(uint16_t reg, uint16_t val)
{
    if (s_fifo_cnt >= CMD_FIFO_DEPTH)
    {
        s_fifo_head = (s_fifo_head + 1U) % CMD_FIFO_DEPTH;  /* 覆盖最旧 */
        s_fifo_cnt--;
        BB_Log(BB_EV_CMD_DROP, reg);
    }
    s_cmd_fifo[s_fifo_tail].reg = reg;
    s_cmd_fifo[s_fifo_tail].val = val;
    s_fifo_tail = (s_fifo_tail + 1U) % CMD_FIFO_DEPTH;
    s_fifo_cnt++;
}

/* 事件检测用的上次状态（GW_Poll 里对比沿变，变化才入事件队列） */
static uint8_t  s_last_online = 0xFF;
static uint16_t s_last_fault  = 0xFFFF;
static uint8_t  s_last_pc_down = 0;     /* PC 心跳 DOWN 上次状态(沿变检测) */

/* ====================== 故障黑匣子 (系统级安全, 持久化到 W25Q64) ======================
 * W25Q64 实体在 F407 侧(不在 F103)。记录 F103 故障/掉线、PC 掉线 等事件,
 * 每次新事件整块 SaveSafe 到扇区 BB_SECTOR, 掉电不丢, 供事后排查。
 * 环形缓冲 BB_MAX 条; 上电从 W25Q64 恢复累计计数与最近事件。 */
#define BB_MAX        16
#define BB_SECTOR     0x001000UL   /* W25Q64 扇区1, 与 FLASH_SelfTest 用扇区错开 */
/* BB_EV_* 事件码定义已上移 modbus_gateway.h (Watchdog_Task 等系统组件需用) */

typedef struct { uint8_t code; uint8_t _pad; uint16_t val; uint32_t ts; } bb_ev_t;
typedef struct { uint16_t total; uint8_t head; uint8_t _pad; bb_ev_t ev[BB_MAX]; } bb_store_t;

static bb_ev_t   s_bb[BB_MAX];
static uint8_t   s_bb_head = 0;
static uint16_t  s_bb_total = 0;
static uint8_t   s_bb_inited = 0;
static uint8_t   s_bb_last_code = 0;
static uint32_t  s_bb_last_ts = 0;

/* 把当前环形缓冲 + 累计计数整块写入 W25Q64(扇区级掉电保护) */
static void bb_persist(void)
{
    if (!s_bb_inited) return;   /* W25Q64 未就绪不写, 避免误操作 */

    /* ★ 健康检查: 面包板软 SPI 时序敏感, W25Q64 偶发不应答。
       若 ReadID 厂商字节非 0xEF, 直接跳过持久化 —— 否则下方
       W25Q64_SaveSafe 的 EraseSector 会触发 w25_wait_busy 忙等 ~2s,
       占满 BB_Task(Low 优先级) 拖慢整机响应。掉电保护偶尔丢几条
       黑匣子记录, 远比重启/卡顿整机可接受。 */
    if ((W25Q64_ReadID() >> 8) != 0xEF) return;

    bb_store_t s;
    s.total = s_bb_total;
    s.head  = s_bb_head;
    memcpy(s.ev, s_bb, sizeof(s_bb));
    W25Q64_SaveSafe(BB_SECTOR, (uint8_t *)&s, sizeof(s));
}

/* 上电首次从 W25Q64 恢复(失败则视为空, 从头累计) */
static void bb_load(void)
{
    bb_store_t s; uint16_t len;
    if (W25Q64_LoadSafe(BB_SECTOR, (uint8_t *)&s, sizeof(s), &len) == 0)
    {
        s_bb_total = s.total;
        s_bb_head  = (s.head < BB_MAX) ? s.head : 0;
        memcpy(s_bb, s.ev, sizeof(s_bb));
    }
    s_bb_inited = 1;
}

/* 记录一条事件: 立即塞队列, 不阻塞调用者；持久化由 BB_Task_Entry 异步处理。
 * 队列满则丢(留 short-circuit 防止累加阻塞)。 */
void BB_Log(uint8_t code, uint16_t val)
{
    if (!s_bb_inited) return;
    bb_qev_t e = { .code = code, .val = val, .ts = HAL_GetTick() / 1000U };
    if (g_q_bb) (void)osMessageQueuePut(g_q_bb, &e, 0, 0);
}

/* BB_Task 入口: 阻塞等黑匣子事件队列, 攒批 + 节流 1s 后持久化到 W25Q64。
 * 优先级 Low (低于 UI), 即使擦扇区 ~30~150ms 也不卡 UI/PC 链路;
 * 事件计数/最近事件/时间戳 立即写镜像寄存器, PC 端 Modbus Poll 实时可见。 */
void BB_Task_Entry(void *arg)
{
    (void)arg;
    bb_qev_t e;
    uint32_t last_persist = 0;
    for (;;)
    {
        WD_Task_Alive(WD_TASK_BB);   /* 心跳上报 (系统服务层) */

        /* ★ 必须用有限超时(1s)而非 osWaitForever:
             若阻塞等待, 系统稳定期无黑匣子事件时本任务永不醒来,
             心跳无法刷新 → 被 wd_task 误判超时 → 整机复位。 */
        if (osMessageQueueGet(g_q_bb, &e, NULL, 1000U) != osOK)
            continue;

        /* 更新环形缓冲 */
        bb_ev_t *p = &s_bb[s_bb_head];
        p->code = e.code;
        p->val  = e.val;
        p->ts   = e.ts;
        s_bb_head = (s_bb_head + 1U) % BB_MAX;
        if (s_bb_total < 0xFFFF) s_bb_total++;
        s_bb_last_code = e.code;
        s_bb_last_ts   = e.ts;

        /* 立即同步镜像寄存器（PC 可随时读到最新条目） */
        g_gw_regs[R_GW_LOG_CNT]  = s_bb_total;
        g_gw_regs[R_GW_LOG_LAST] = s_bb_last_code;
        g_gw_regs[R_GW_LOG_TS]   = s_bb_last_ts;

        /* 节流持久化: 1s 最多 1 次（即使队列里积压 N 条, 也只写最后这一组） */
        uint32_t now = HAL_GetTick();
        if ((now - last_persist) >= 1000U)
        {
            last_persist = now;
            bb_persist();
        }
    }
}

/* 中断回调里用的单字节接收变量（CubeMX 生成的 IT 文件通过 GW_UART_RxISR 访问） */
uint8_t s_usart1_rx_byte;
uint8_t s_usart3_rx_byte;

/* ====================== 初始化 ====================== */
void GW_Init(void)
{
    memset(g_gw_regs, 0, sizeof(g_gw_regs));
    ringbuf_init(&s_rb1);
    ringbuf_init(&s_rb3);

    /* 板载 LED0/1 (PF9/PF10) 由 CubeMX MX_GPIO_Init() 统一初始化, 此处不重复 */

    /* 注意: 不在此启动 HAL_UART_Receive_IT —— FreeRTOS 队列在 MX_FREERTOS_Init
     * 里才创建, 若调度器启动前就有 RXNE 中断触发, ISR 里 PutFromISR 会拿到
     * 空队列句柄直接崩。RXNE 启动移到 GW_StartRx(), 由 gw_task 入口调用。 */
}

void GW_StartRx(void)
{
    HAL_UART_Receive_IT(&huart1, &s_usart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart3, &s_usart3_rx_byte, 1);
}

void GW_UART_RxISR(UART_HandleTypeDef *huart)
{
    uint16_t item;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* osMessageQueueId_t 在 ST 适配里就是 QueueHandle_t 转 void*, 这里 cast 回来 */
    QueueHandle_t hq = (QueueHandle_t) g_q_rx;

    if (huart->Instance == USART1)
    {
        /* 先重新武装接收（最紧要），再把字节投进队列 */
        HAL_UART_Receive_IT(&huart1, &s_usart1_rx_byte, 1);
        item = (1u << 8) | s_usart1_rx_byte;
        xQueueSendFromISR(hq, &item, &xHigherPriorityTaskWoken);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(&huart3, &s_usart3_rx_byte, 1);
        item = (3u << 8) | s_usart3_rx_byte;
        xQueueSendFromISR(hq, &item, &xHigherPriorityTaskWoken);
    }

    /* 若有更高优先级任务因入队被唤醒, 立即切换过去, 不在 ISR 里拖沓 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* UART 错误自恢复(自动重连): 总线上偶发一帧错误(噪声/波特抖动→ORE/FE/NE) 时,
 * HAL 进本回调而不进 RxCpltCallback, 导致 Receive_IT 不再被武装 → RX 永久停收,
 * 只能整机复位。这里清错误标志并重新武装两路 UART 接收, 通信自动恢复。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &s_usart1_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(&huart3, &s_usart3_rx_byte, 1);
    }
}

/* gw_task 上下文调用: 把从字节队列取出的 {port,byte} 喂给对应接收缓冲 */
void GW_FeedRxByte(uint8_t port, uint8_t ch)
{
    if (port == 1)
    {
        ringbuf_put(&s_rb1, ch);
        s_usart1_last_tick = HAL_GetTick();
        s_pc_seen = 1;                 /* 收到 PC 任意字节即记"已连过" */
    }
    else
    {
        ringbuf_put(&s_rb3, ch);
        s_usart3_last_tick = HAL_GetTick();
    }
}

/* ====================== 主站事务（对 F103） ====================== */
/* 把字节队列里的 {port,byte} 全部喂给接收缓冲（gw_task 单线程内调用）。
 * ★ 必须在 gw_master_read/write 等待应答期间持续调用 —— 否则 F103 应答
 *   卡在队列里, ringbuf 永远是空, 每次读都 120ms 超时 → F103 被误判掉线。 */
static void gw_drain_rx_queue(void)
{
    uint16_t item;
    while (osMessageQueueGet(g_q_rx, &item, NULL, 0) == osOK)
    {
        GW_FeedRxByte((uint8_t)(item >> 8), (uint8_t)(item & 0xFF));
    }
}

/* 读 F103 保持寄存器，返回提取到的寄存器数，<0 表示失败 */
static int gw_master_read(uint16_t reg, uint16_t count, uint16_t *out)
{
    uint8_t req[8];
    uint16_t n = mb_build_read_req(req, F103_ADDR, reg, count);

    /* 清空接收缓冲，避免旧数据干扰 */
    s_rb3.tail = s_rb3.head;

    HAL_UART_Transmit(&huart3, req, n, 100);

    uint8_t frame[MB_MAX_FRAME];
    uint16_t flen = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < MASTER_TIMEOUT_MS)
    {
        gw_drain_rx_queue();   /* ★ 等待应答期间持续消费队列 → ringbuf */
        uint8_t c;
        while (ringbuf_get(&s_rb3, &c))
        {
            if (flen < MB_MAX_FRAME) frame[flen++] = c;
            if (flen >= 5)
            {
                uint8_t expect = 5 + frame[2];   /* 读响应长度 = 5 + 字节数 */
                if (flen >= expect)
                {
                    if (mb_check_frame(frame, expect))
                        return mb_parse_read_resp(frame, expect, out, count);
                    else
                        return -1;  /* CRC 错误 */
                }
            }
        }
        osDelay(1);   /* ★★ 让出 CPU: 否则 120ms 忙等会饿死低优先级 ui_task(OLED 渲染) */
    }
    return -1;  /* 超时 */
}

/* 写单个寄存器到 F103，返回 0 成功 */
static int gw_master_write(uint16_t reg, uint16_t val)
{
    uint8_t req[8];
    uint16_t n = mb_build_write_single_req(req, F103_ADDR, reg, val);

    s_rb3.tail = s_rb3.head;
    HAL_UART_Transmit(&huart3, req, n, 100);

    uint8_t frame[MB_MAX_FRAME];
    uint16_t flen = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < MASTER_TIMEOUT_MS)
    {
        gw_drain_rx_queue();   /* ★ 同上: 等待应答期间持续消费队列 */
        uint8_t c;
        while (ringbuf_get(&s_rb3, &c))
        {
            if (flen < MB_MAX_FRAME) frame[flen++] = c;
            if (flen >= 8)
            {
                if (mb_check_frame(frame, flen))
                    return 0;
            }
        }
        osDelay(1);   /* ★★ 让出 CPU, 避免 120ms 忙等饿死 ui_task */
    }
    return -1;
}

/* ====================== 从站回调（对 PC） ====================== */
static uint16_t gw_read_reg(uint16_t reg)
{
    if (reg < GW_REG_COUNT) return g_gw_regs[reg];
    return 0;
}
 

static int gw_write_reg(uint16_t reg, uint16_t val)
{
    switch (reg)
    {
        case R_GW_MOTOR_TARGET:   /* PC 下发目标速度 (0..999, 越界拒绝防异常) */
            if (val > 999) return -1;
            g_gw_regs[reg] = val;
            s_pending_target = 1;       /* COALESCE: 只留最新, 不入 FIFO */
            s_pending_target_val = val;
            return 0;
        case R_GW_CMD:            /* PC 下发系统指令 (只接受已定义 bit, 未知 bit 拒绝) */
            if (val & ~(CMD_START | CMD_STOP | CMD_RESET)) return -1;
            g_gw_regs[reg] = val;
            cmd_fifo_push(R_F103_CTRL, val);   /* DELIVER-ALL: 入队补发 */
            return 0;
        case R_GW_SORT_CMD:       /* PC 远程触发分拣 (0无 1→A 2→B); 网关透传 F103 reg5, 走补发FIFO */
            if (val > 2) return -1;
            g_gw_regs[reg] = val;
            cmd_fifo_push(R_F103_SORT_CMD, val);   /* DELIVER-ALL: 入队补发 */
            return 0;
        default:
            return -1;   /* 其余寄存器只读，返回非法地址异常 */
    }
}

/* ====================== 主循环 ====================== */
void GW_Poll(void)
{
    uint32_t now = HAL_GetTick();

    /* 黑匣子上电懒加载: 首次进入从 W25Q64 恢复累计计数与最近事件
       (W25Q64 已在 APP_MAIN_Init 的 FLASH_SelfTest 初始化, 此处已就绪) */
    if (!s_bb_inited) bb_load();

    /* #1: 看门狗复位事件迁移 —— 上一次 IWDG 复位前把事件码写进了 BKPSRAM,
       此刻在稳定上下文读出并正式写黑匣子(掉电不丢)。复位/掉电都丢不了。
       TakeEvent 自带清零, 配合 s_bkp_migrated 仅在本固件运行期内迁移一次。 */
    {
        static uint8_t s_bkp_migrated = 0;
        if (!s_bkp_migrated)
        {
            uint8_t ev = 0;
            if (WD_Bkp_TakeEvent(&ev)) { BB_Log(ev, 0); s_bkp_migrated = 1; }
        }
    }

    /* ---------- 1) 处理 PC（USART1）从站请求 ---------- */
    if (ringbuf_avail(&s_rb1) > 0 &&
        (now - s_usart1_last_tick) >= 5)   /* 5ms 静默判定一帧结束 */
    {
        uint8_t frame[MB_MAX_FRAME];
        uint16_t flen = 0;
        uint8_t c;
        while (ringbuf_get(&s_rb1, &c) && flen < MB_MAX_FRAME)
            frame[flen++] = c;

        if (flen >= 4 && mb_check_frame(frame, flen))
        {
            uint8_t resp[MB_MAX_FRAME];
            uint16_t rn = mb_slave_handle(frame, flen, resp, MB_MAX_FRAME,
                                          GW_ADDR, gw_read_reg, gw_write_reg);
            if (rn) HAL_UART_Transmit(&huart1, resp, rn, 100);
        }
    }

    /* ---------- 2) 主站轮询 F103（USART3） ---------- */
    if (now - s_last_poll >= MASTER_PERIOD_MS)
    {
        s_last_poll = now;

        /* 先处理待转发的写请求 (在线才发, 失败留待下轮重试, 断链不丢) */
        if (s_pending_target)
        {
            if (gw_master_write(R_F103_TARGET, s_pending_target_val) == 0)
                s_pending_target = 0;   /* COALESCE: 成功才清合并槽 */
        }
        /* DELIVER-ALL: 逐条发 FIFO 队首, 成功才出队(失败留队, 断链重连后 RESYNC 补发) */
        while (s_fifo_cnt > 0)
        {
            cmd_fifo_ent_t *e = &s_cmd_fifo[s_fifo_head];
            if (gw_master_write(e->reg, e->val) != 0) break;  /* 本条失败, 后续留待下轮 */
            s_fifo_head = (s_fifo_head + 1U) % CMD_FIFO_DEPTH;
            s_fifo_cnt--;
        }

        /* 读回 F103 全部寄存器并镜像到网关 */
        uint16_t f103[F103_REG_COUNT];
        int r = gw_master_read(0, F103_REG_COUNT, f103);
        if (r > 0)
        {
            s_offline_cnt = 0;
            s_f103_online = 1;
            g_gw_regs[R_GW_MOTOR_PWM]  = f103[R_F103_ACT_PWM];
            g_gw_regs[R_GW_MOTOR_STATUS]= f103[R_F103_STATUS];
            g_gw_regs[R_GW_MOTOR_FAULT] = f103[R_F103_FAULT];
            g_gw_regs[R_GW_MAT_CNT]    = f103[R_F103_MAT_CNT];
            g_gw_regs[R_GW_SENSE_A]    = f103[R_F103_SENSE_A];
            g_gw_regs[R_GW_SENSE_B]    = f103[R_F103_SENSE_B];
            g_gw_regs[R_GW_POST]       = f103[R_F103_POST];   /* POST 自检结果镜像 */
            g_gw_regs[R_GW_SYS_STATUS] = BIT_LINK |
                ((f103[R_F103_STATUS] & BIT_RUN) ? 0x0002 : 0);  /* SYS_STATUS bit1=运行 */
        }
        else
        {
            /* 115200 面包板链路偶发误码会让单次读失败；连续 3 次才判掉线，
               避免一次毛刺就把 LED1 闪灭 / OLED 数值清零（保留上次正常值）。 */
            if (s_offline_cnt < 3) s_offline_cnt++;
            if (s_offline_cnt >= 3)
            {
                s_f103_online = 0;
                g_gw_regs[R_GW_SYS_STATUS]  &= ~BIT_LINK;
                /* F103 失联: 清掉陈旧 RUN 位, 仅置 GW_DOWN, 让 PC 经 COM14 读到
                   与安全态一致的 0x08 (F103 侧独立检测已自行停电机)。
                   否则 R_GW_MOTOR_STATUS 停在最后一次成功的旧值(1), PC 看不到掉线。 */
                g_gw_regs[R_GW_MOTOR_STATUS] = BIT_GW_DOWN;
            }
        }
    }

    /* ---------- 3) 状态沿变 → 事件队列（gw_task→alert_task） ---------- */
    if (s_f103_online != s_last_online)
    {
        uint8_t was_online = s_last_online;
        s_last_online = s_f103_online;
        uint8_t ev = s_f103_online ? GW_EV_F103_UP : GW_EV_F103_DOWN;
        osMessageQueuePut(g_q_ev, &ev, 0, 0);
        BB_Log(s_f103_online ? BB_EV_F103_ONLINE : BB_EV_F103_OFFLINE, 0);

        /* RESYNC: F103 重新上线 → 触发断链期间下行指令重放
         * (实际发送交给上方轮询期 FIFO 逻辑, 逐条平滑发出, 避免一次性阻塞 gw_task) */
        if (s_f103_online && !was_online)
        {
            BB_Log(BB_EV_RESEND_START, s_fifo_cnt);
            s_resync_pending = (s_fifo_cnt > 0) ? 1 : 0;
        }
    }
    /* RESYNC 完成: 补发队列由非空 → 空 (且曾置 pending) */
    if (s_resync_pending && s_fifo_cnt == 0)
    {
        s_resync_pending = 0;
        BB_Log(BB_EV_RESEND_DONE, 0);
    }
    uint16_t fault = g_gw_regs[R_GW_MOTOR_FAULT];
    if (fault != s_last_fault)
    {
        s_last_fault = fault;
        uint8_t ev = (fault != FAULT_NONE) ? GW_EV_FAULT : GW_EV_FAULT_CLEAR;
        osMessageQueuePut(g_q_ev, &ev, 0, 0);
        BB_Log((fault != FAULT_NONE) ? BB_EV_F103_FAULT : BB_EV_FAULT_CLEAR, fault);
    }

    /* ---------- 4) 板载 LED1 (PF10) 链路指示 ---------- */
    /* 板载 LED 为低电平亮(active-low)：在线 → RESET(亮)；掉线 → SET(灭) */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10,
                      s_f103_online ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* ---------- 5) PC 心跳超时检测（通信级安全） ---------- */
    /* s_usart1_last_tick 在收到 PC 帧任意字节时由 GW_FeedRxByte 刷新。
       三态: ① 从未连过(s_pc_seen=0)→ PC:-- (待机, 不算故障)
             ② 连过但 >3s 无请求    → PC:DOWN (BIT_PC_DOWN, 真失联)
             ③ 连过且近期有请求      → PC:OK   (清 BIT_PC_DOWN)
       放最后, 避免被上方 F103 在线分支的整体赋值(SYS_STATUS=BIT_LINK|...)覆盖。 */
    if (!s_pc_seen)
    {
        g_gw_regs[R_GW_SYS_STATUS] &= ~(BIT_PC_DOWN | BIT_PC_SEEN);
    }
    else if ((now - s_usart1_last_tick) > PC_HEARTBEAT_MS)
    {
        g_gw_regs[R_GW_SYS_STATUS] |= (BIT_PC_DOWN | BIT_PC_SEEN);
    }
    else
    {
        g_gw_regs[R_GW_SYS_STATUS] = (g_gw_regs[R_GW_SYS_STATUS] | BIT_PC_SEEN) & ~BIT_PC_DOWN;
    }

    /* 补发队列状态位: FIFO 非空 → PC 可见"指令积压" */
    if (s_fifo_cnt > 0)
        g_gw_regs[R_GW_SYS_STATUS] |= BIT_RESEND;
    else
        g_gw_regs[R_GW_SYS_STATUS] &= ~BIT_RESEND;

    /* ---------- 5b) PC 心跳沿变 → 事件队列（gw_task→alert_task） ---------- */
    /* ★ 1s 稳定确认: pyserial 在 Windows 上 ser.write 偶发阻塞 12s+,
       阻塞期间 USART1 静默 → 阈值被击穿判 PC_DOWN → 阻塞释放 1s 内
       USART1 重新有字节 → PC_UP 翻转。这种 1s 内 DOWN↔UP 反复翻转
       是驱动层偶发问题, 业务上无意义 (PC 仍在线, 只是 GUI 发不出
       请求)。所以: 状态翻转时记到 pending, 1s 内若再翻转则丢弃;
       1s 后仍稳定才入队 + 写黑匣子。 */
    uint32_t now_tick = HAL_GetTick();
    uint8_t pc_down_now = (g_gw_regs[R_GW_SYS_STATUS] & BIT_PC_DOWN) ? 1 : 0;
    if (pc_down_now != s_pc_down_pending)
    {
        s_pc_down_pending = pc_down_now;
        s_pc_change_tick  = now_tick;
    }
    if (pc_down_now != s_last_pc_down && (now_tick - s_pc_change_tick) >= 1000U)
    {
        s_last_pc_down = pc_down_now;
        uint8_t ev = pc_down_now ? GW_EV_PC_DOWN : GW_EV_PC_UP;
        osMessageQueuePut(g_q_ev, &ev, 0, 0);
        BB_Log(pc_down_now ? BB_EV_PC_DOWN : BB_EV_PC_UP, 0);
    }

    /* ---------- 6) 黑匣子快照镜像到寄存器(供 PC 经 Modbus 读取) ---------- */
    g_gw_regs[R_GW_LOG_CNT]  = s_bb_total;
    g_gw_regs[R_GW_LOG_LAST] = s_bb_last_code;
    g_gw_regs[R_GW_LOG_TS]   = (uint16_t)(s_bb_last_ts & 0xFFFF);
}

/* UI/按键触发系统指令：直接写入镜像并置待转发标志，
 * 下一个 GW_Poll 周期会经 USART3 转发给 F103 (R_F103_CTRL) */
void GW_RequestCmd(uint16_t cmd)
{
    g_gw_regs[R_GW_CMD] = cmd;
    cmd_fifo_push(R_F103_CTRL, cmd);   /* 走统一下行补发通路, 断链不丢 */
}
