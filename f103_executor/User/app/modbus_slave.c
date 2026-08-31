/* ============================================================================
 * F103 端 Modbus RTU 从站实现
 *
 *  - USART1 (PA9/PA10) : 调试串口 (printf + SSCOM 命令)
 *      → 字节入队通过 APP_UART1_RxByte() 转给 app_main.c 的原环形缓冲
 *  - USART2 (PA2/PA3)  : Modbus RTU 从站 (addr=0x01)
 *      → 本文件 ringbuf + 静默分帧 + mb_slave_handle
 *
 * 关键铁律:
 *  1) HAL_UART_Receive_IT 必须在 MX_USARTx_UART_Init() 之后调用,否则
 *     huart->Instance 为 NULL,HAL 静默返回 HAL_ERROR,RXNE 中断永远不武装。
 *  2) HAL_UART_RxCpltCallback 默认是 __weak 空函数,本文件必须有一个 strong
 *     实现,否则 USART1 入队逻辑会被链接器当 dead code 删除(同 F407 那次坑)。
 *  3) mb_slave_handle 接受 my_addr 入参,非本机地址不响应(避免总线上被抢答)。
 * ==========================================================================*/

#include "modbus_slave.h"
#include "modbus_rtu.h"
#include "mb_regmap.h"
#include "ringbuf.h"
#include "app_motor_ctrl.h"
#include "bsp_adc.h"   /* BSP_ADC_GetRaw() - 光敏/反射红外 ADC */
#include "bsp_servo.h" /* 舵机限位驱动 (SORT_CMD 接舵机) */

#include "main.h"
#include "usart.h"     /* huart1 (调试) + huart2 (Modbus, .ioc 配 USART2 后才有) */
#include "stm32f1xx_hal.h"

#include <string.h>

extern TIM_HandleTypeDef htim1;   /* TIM1 CH1 PWM 在 bsp_motor.c 里定义 */

/* ==================== 私有状态 ==================== */
/* USART2 (Modbus) 改 DMA+IDLE: 硬件 DMA 自动收, 收到一帧 (IDLE 或满) 触发回调。
   USART1 (调试) 仍走 RXNE+ringbuf, 调试 printf 不能用 DMA 抢字符。 */
#define USART2_RX_BUF_SIZE   64U
static uint8_t  s_usart2_rx_buf[USART2_RX_BUF_SIZE];   /* DMA 目标缓冲 (CubeMX 配) */
static volatile uint8_t  s_usart2_rx_rdy = 0;          /* RxEventCallback 置 1, Poll 消费后清 0 */
static volatile uint16_t s_usart2_rx_len = 0;          /* 本帧实际字节数 */
static uint32_t s_last_master_tick;     /* 最后收到 F407 主站合法请求帧的 tick (静默检测) */
#define GW_SILENCE_MS   2000U          /* 超过此时间未收到主站轮询 → 判定网关掉线 */
static uint8_t  s_uart1_rx_byte;       /* USART1 HAL_UART_Receive_IT 单字节缓冲 */

/* Modbus 寄存器镜像 (本端视角, 即 F103 内部状态的对外映射) */
static uint16_t    ms_regs[F103_REG_COUNT];

/* 写命令缓存: PC 下发但还没同步到应用层的值 */
static uint8_t     s_pending_target = 0;
static uint16_t    s_pending_target_val = 0;
static uint8_t     s_pending_ctrl = 0;
static uint16_t    s_pending_ctrl_val = 0;

/* 应用层运行时状态 (用于回填 STATUS 寄存器) */
static uint8_t     s_running = 0;
static uint8_t     s_sorting = 0;
static uint16_t    s_fault   = FAULT_NONE;   /* 位掩码: 见 mb_regmap.h FAULT_* */

/* 业务计数 (从 EXTI/舵机回调累加, 主循环读出到寄存器) */
static volatile uint16_t s_mat_cnt = 0;     /* 对射红外 PA12 EXTI12↓ 累加 */
static volatile uint16_t s_sort_a_cnt = 0;  /* 物料分入 A 料道 计数 */
static volatile uint16_t s_sort_b_cnt = 0;  /* 物料分入 B 料道 计数 */

/* 自动分拣状态机 (本地自动, 不依赖 PC):
 *   物料经 PA12 对射到达 → NotifyMatArrived 读光敏 PB0 分类 → 置 s_arrive+slot
 *   Poll 消费: 立即打舵(A=1/B=2) → 保持 SORT_HOLD_MS → 回中并 NotifySortDone 计数 */
#define SORT_LIGHT_THRESH_DFLT   2048U   /* 光敏分类阈值默认(占位, 串口 TL= 实时调) */
#define SORT_RECHECK_THRESH_DFLT 2500U   /* PB1 反射红外复检阈值默认(占位, 串口 TR= 实时调) */
#define SORT_HOLD_MS       600U    /* 打舵后保持时间, 让物料滑入料道 */
#define ARRIVE_BOOT_GRACE_MS  1000U  /* 上电去抖: 启动后 1s 内的 PA12 边沿视为瞬态(防 F103 上电瞬态误计) */
#define ARRIVE_RETRIG_MS      500U  /* 重触发去抖: 距上次有效触发 <500ms 视为抖动/慢速手扫多边沿, 跳过 */
#define TEMP_OVER_ADC       1500U   /* PA4 热敏超温停机阈值(原始ADC)。本工程 NTC 发热→阻值↓→分压↓→ADC↓,
                                        故用 < 判断(见下方)。若你的分压是发热→ADC↑, 把 < 改成 >。占位值需实测标定 */
#define TEMP_HYST_ADC       150U    /* 超温滞回, 防临界抖动反复启停 */
static uint16_t s_sort_light_thresh   = SORT_LIGHT_THRESH_DFLT;   /* 光敏分类阈值(运行期, 串口 TL= 调) */
static uint16_t s_sort_recheck_thresh = SORT_RECHECK_THRESH_DFLT; /* PB1 复检阈值(运行期, 串口 TR= 调) */
static volatile uint8_t  s_arrive      = 0;  /* PA12 中断置位的"物料到达"事件 */
static volatile uint8_t  s_arrive_slot = 0;  /* 分类结果: 1=A, 2=B */
static uint8_t           s_sort_busy   = 0;  /* Poll 内打舵状态: 1=已打舵待回中 */
static uint32_t          s_sort_tick   = 0;  /* 打舵时刻 (HAL_GetTick) */
static uint32_t          s_last_arrive_tick = 0;  /* 上次有效 PA12 触发时间 (HAL_GetTick), 去抖用 */
static uint8_t           s_over_temp    = 0;  /* PA4 超温锁存(带滞回), 触发后紧急停电机 */
static uint8_t           s_recheck_ok   = 0;  /* 本次分拣 PB1 复检是否通过(分拣窗口内采样) */
static uint8_t           s_recheck_last = 0;  /* 上次分拣复检结果状态位, 持续显示到下次分拣 */

/* ADC 周期刷新节流 (StartOnce 内部跑 3 次 HAL_ADC_Start/Poll/Stop ~10ms,
   不要每次 Poll 都跑,否则 10ms 主循环会被拖到 30+ms, Modbus 应答卡) */
static uint32_t    s_last_adc_tick = 0;
#define ADC_SCAN_PERIOD_MS   50U

/* 传感器断线检测(去抖): 分压型 ADC 传感器开路(悬空/碰VCC→raw≈4095)或
   短路(碰GND→raw≈0) 都表现为"卡在极端值"。连续 N 次异常置位、连续 N 次
   正常清零, 避免瞬时抖动误报。对射红外 PA12 是数字量(无物料时常恒电平),
   不判断线(易误报)。 */
#define SEN_OPEN_THR    4000    /* raw > 此值 视为开路/上拉短路 */
#define SEN_SHORT_THR   80      /* raw < 此值 视为短路到 GND */
#define SEN_DEBOUNCE    5       /* 连续 5 次异常置位 / 连续 5 次正常清 */
static uint8_t s_deb_temp = 0;
static uint8_t s_deb_light = 0;
static uint8_t s_deb_reflect = 0;

/* POST 上电自检结果(由 app_main.c 开机跑完 POST 后上报, 之后恒定) */
static uint16_t s_post_mask = 0;

/* 单路 ADC 断线去抖: 异常累加、正常递减, 达阈值置/清对应 FAULT 位 */
static void sensor_wire_check(uint16_t raw, uint8_t *deb, uint16_t fault_bit)
{
    if (raw < SEN_SHORT_THR || raw > SEN_OPEN_THR)
    {
        if (*deb < SEN_DEBOUNCE) (*deb)++;
    }
    else
    {
        if (*deb > 0) (*deb)--;
    }
    if      (*deb >= SEN_DEBOUNCE) s_fault |=  fault_bit;
    else if (*deb == 0)            s_fault &= ~fault_bit;
}

/* 默认启动占空比: 电机只有 2 根线(无轴编码器), 走开环 PWM (0~999)。
   5V 供电扭矩有限, 取中等值。PID 闭环需要电机轴编码器(非 EC11 旋钮), 暂无。 */
#define MOTOR_START_PWM     400U

void MODBUS_SLAVE_NotifyMatArrived(void)
{
    uint32_t now = HAL_GetTick();

    /* 上电去抖: 启动后 1s 内的边沿视为瞬态, 跳过 (防 F103 上电瞬态误计物料) */
    if (now < ARRIVE_BOOT_GRACE_MS) return;
    /* 重触发去抖: 距上次有效触发 <500ms 视为抖动/慢速手扫多边沿, 跳过
       (500ms < SORT_HOLD_MS=600ms, 让一次分拣完整结束再接收下一物料) */
    if ((now - s_last_arrive_tick) < ARRIVE_RETRIG_MS) return;
    s_last_arrive_tick = now;

    s_mat_cnt++;   /* EXTI 中断上下文, 仅 16bit 自增不可重入冲突 */
    /* 自动分拣: 仅在空闲时接受新物料, 避免连续触发互相打断。
       分类依据 = 光敏 PB0 (ADC_RANK_LIGHT_SEN), 阈值 s_sort_light_thresh(运行期可 TL= 调)。 */
    if (!s_sort_busy)
    {
        uint16_t raw = BSP_ADC_GetRaw(ADC_RANK_LIGHT_SEN);
        s_arrive_slot = (raw > s_sort_light_thresh) ? 1 : 2;
        s_arrive = 1;
    }
}

/* POST 自检结果上报接口: app_main.c 开机调用 post_run() 后把结果传进来 */
void MODBUS_SLAVE_ReportPost(uint16_t mask)
{
    s_post_mask = mask;
}

void MODBUS_SLAVE_NotifySortDone(uint8_t slot)
{
    if (slot == 1) s_sort_a_cnt++;
    else if (slot == 2) s_sort_b_cnt++;
    /* 其它值忽略 */
}

/* 分拣/复检阈值运行期标定访问器 (#5) */
void MODBUS_SLAVE_SetLightThresh(uint16_t v)    { s_sort_light_thresh = v; }
void MODBUS_SLAVE_SetRecheckThresh(uint16_t v)  { s_sort_recheck_thresh = v; }
uint16_t MODBUS_SLAVE_GetLightThresh(void)      { return s_sort_light_thresh; }
uint16_t MODBUS_SLAVE_GetRecheckThresh(void)    { return s_sort_recheck_thresh; }

/* ==================== 中断入口 (USART1 + USART2 IDLE) ==================== */
/* USART1 字节 → APP_UART1_RxByte (给 app_main.c 原环形缓冲, 调试口不走 DMA)
   USART2 → HAL_UARTEx_ReceiveToIdle_DMA, 一帧结束由 HAL_UARTEx_RxEventCallback 接 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        APP_UART1_RxByte(s_uart1_rx_byte);
        HAL_UART_Receive_IT(&huart1, &s_uart1_rx_byte, 1);
    }
    /* USART2 不在此处, 改走 HAL_UARTEx_RxEventCallback (DMA+IDLE) */
}

/* USART2 DMA+IDLE 帧结束回调: 每次一帧收完 (IDLE 或缓冲半/全满) 触发。
   必须在回调里重新调 HAL_UARTEx_ReceiveToIdle_DMA 武装下一帧, 否则
   只收一次。F103 的 HAL 实现里, 满缓冲回调和 IDLE 回调共用本函数。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;

    s_usart2_rx_len = Size;       /* 本帧实际字节数 (DMA 已自动停止) */
    s_usart2_rx_rdy = 1;          /* 通知 Poll 消费 */
    /* 立即重新武装下一帧, 避免漏收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_usart2_rx_buf, USART2_RX_BUF_SIZE);
}

/* UART 错误自恢复(自动重连): 同 F407。一帧错误(ORE/FE/NE) 进本回调。
   USART2 的错误会被 HAL_UARTEx_RxEventCallback 的 ErrorCode 参数捕获,
   这里也清 USART1 的错误, USART2 错误也清以防万一。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &s_uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        /* 错误后重新武装 DMA+IDLE */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_usart2_rx_buf, USART2_RX_BUF_SIZE);
    }
}

/* ==================== 初始化 ==================== */
void MODBUS_SLAVE_Init(void)
{
    memset(ms_regs, 0, sizeof(ms_regs));
    s_usart2_rx_rdy = 0;
    s_usart2_rx_len = 0;
    s_last_master_tick = HAL_GetTick();

    /* 启动 USART2 接收 (DMA+IDLE 一体化: HAL_UARTEx_ReceiveToIdle_DMA
       内部启用 USART2 IDLE 中断 + DMA1_Ch6 中断 + 启动 DMA 接收,
       IDLE 检测到一帧结束或缓冲半/全满 → HAL_UARTEx_RxEventCallback 回调)。
       CubeMX 配 USART2 → DMA1_Ch6 (F103 固定映射) 后 hdma_usart2_rx 自动生成。 */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);   /* 优先级低于 USART1 (0,0) */
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_usart2_rx_buf, USART2_RX_BUF_SIZE);

    /* USART1 也由本入口接管 (USART1 NVIC 在 app_main.c 启动,这里只需挂首次接收)。
       注意:app_main.c 不再重写 HAL_UART_RxCpltCallback,字节统一从本入口进。 */
    HAL_UART_Receive_IT(&huart1, &s_uart1_rx_byte, 1);
}

/* ==================== 读 / 写寄存器回调 ==================== */
static uint16_t ms_read_reg(uint16_t reg)
{
    if (reg < F103_REG_COUNT) return ms_regs[reg];
    return 0;
}

static int ms_write_reg(uint16_t reg, uint16_t val)
{
    if (reg >= F103_REG_COUNT) return -1;

    switch (reg)
    {
        case R_F103_TARGET:
            /* PC 下发目标 PWM (0..999) */
            if (val > 999) val = 999;
            ms_regs[reg] = val;
            s_pending_target = 1;
            s_pending_target_val = val;
            return 0;

        case R_F103_SORT_CMD:
            /* 分拣指令: 0=回中 1=A 料道 2=B 料道
               接到舵机, 角度经 bsp_servo 限位 clamp[0,180] */
            if (val > 2) return -1;
            ms_regs[reg] = val;
            s_sorting = (val != 0);
            s_sort_busy = 0;   /* 手动指令接管, 取消进行中的自动分拣 */
            BSP_SERVO_Sort(val);   /* 立即执行摆位(主循环上下文, 非中断, 安全) */
            return 0;

        case R_F103_CTRL:
            /* 控制位: bit0=启动 bit1=停止 bit2=复位 */
            ms_regs[reg] = val;
            s_pending_ctrl = 1;
            s_pending_ctrl_val = val;
            return 0;

        default:
            /* 其余寄存器 (ACT_PWM/STATUS/FAULT/MAT_CNT/SENSE_A/B) 只读 */
            return -1;
    }
}

/* ==================== 主循环 ==================== */
void MODBUS_SLAVE_Poll(void)
{
    uint32_t now = HAL_GetTick();

    /* ----- 1) 处理 PC→F103 请求 (USART2, DMA+IDLE 一帧一回调) ----- */
    if (s_usart2_rx_rdy)
    {
        s_usart2_rx_rdy = 0;
        uint16_t flen = (s_usart2_rx_len < MB_MAX_FRAME) ? s_usart2_rx_len : MB_MAX_FRAME;

        if (flen >= 4 && mb_check_frame(s_usart2_rx_buf, flen))
        {
            s_last_master_tick = now;   /* 收到合法主站请求帧 → 网关存活, 刷新时间戳 */
            uint8_t resp[MB_MAX_FRAME];
            uint16_t rn = mb_slave_handle(s_usart2_rx_buf, flen, resp, MB_MAX_FRAME,
                                          F103_ADDR, ms_read_reg, ms_write_reg);
            if (rn) HAL_UART_Transmit(&huart2, resp, rn, 100);
        }
    }

    /* ----- 2) 把待下发的写命令同步到应用层 ----- */
    if (s_pending_target)
    {
        /* PC 下发的目标 PWM → 开环直接驱动电机 (与 O<pwm> 命令等价)。
           电机无轴编码器 (PA0/1 接的是 EC11 手动旋钮, 非测速反馈), 故采用开环 PWM。 */
        APP_MOTOR_CTRL_OpenLoop((uint16_t)s_pending_target_val,
                                (int16_t)(s_pending_target_val > 0 ? 1 : 0));
        s_pending_target = 0;
    }
    if (s_pending_ctrl)
    {
        if (s_pending_ctrl_val & 0x0001)       /* bit0 启动 */
        {
            s_running = 1;
            APP_MOTOR_CTRL_OpenLoop(MOTOR_START_PWM, 1);  /* 开环默认转速, 正转 */
        }
        if (s_pending_ctrl_val & 0x0002)       /* bit1 停止 */
        {
            s_running = 0;
            APP_MOTOR_CTRL_OpenLoop(0, 0);     /* 刹车 */
        }
        if (s_pending_ctrl_val & 0x0004)       /* bit2 复位 */
        {
            s_running = 0;
            s_fault = FAULT_NONE;
            APP_MOTOR_CTRL_ClearFault();        /* 内部已刹车 + 清 PID */
        }
        s_pending_ctrl = 0;
    }

    /* ----- 3) 同步 F103 内部状态到寄存器镜像 -----
     * 先周期性刷新 ADC (StartOnce 内部跑 3 个 Rank + 滑动平均; 50ms 节流避免阻塞主循环)
     * 不调 GetRaw 会永远返回 0 (BSP_ADC_Init 只预填一次窗口) */
    if ((now - s_last_adc_tick) >= ADC_SCAN_PERIOD_MS)
    {
        BSP_ADC_StartOnce();
        s_last_adc_tick = now;

        /* 传感器断线检测: 读三路 ADC 原始值, 去抖后置/清对应 FAULT 位。
           PA4 热敏(Rank0) / PB0 光敏(Rank1) / PB1 反射红外(Rank2)。
           对射红外 PA12 是数字量(无物料时常恒电平), 不判断线(避免误报)。 */
        uint16_t raw_temp    = BSP_ADC_GetRaw(ADC_RANK_POTENTIOMETER);
        uint16_t raw_light   = BSP_ADC_GetRaw(ADC_RANK_LIGHT_SEN);
        uint16_t raw_reflect = BSP_ADC_GetRaw(ADC_RANK_REFLECT_IR);
        sensor_wire_check(raw_temp,    &s_deb_temp,    FAULT_SEN_TEMP);
        /* PA4 热敏 超温停机 (安全): 超阈值 → 置 FAULT_OVERHEAT + 紧急停电机, 带滞回防抖。
           NTC 发热→阻值↓→分压↓→ADC↓, 故用 < 判断; 若硬件分压相反则改 > 并翻转滞回方向。 */
        if (!s_over_temp && raw_temp < TEMP_OVER_ADC)
            s_over_temp = 1;
        else if (s_over_temp && raw_temp > (TEMP_OVER_ADC + TEMP_HYST_ADC))
            s_over_temp = 0;
        if (s_over_temp)
        {
            s_fault |= FAULT_OVERHEAT;
            if (s_running) { s_running = 0; APP_MOTOR_CTRL_OpenLoop(0, 0); }
        }
        else
            s_fault &= ~FAULT_OVERHEAT;
        sensor_wire_check(raw_light,   &s_deb_light,   FAULT_SEN_LIGHT);
        sensor_wire_check(raw_reflect, &s_deb_reflect, FAULT_SEN_REFLECT);
        /* PB1 反射红外 物料复检: 分拣窗口内按 ADC 节拍采样, 物料经过使反射值越线 → 复检通过。
           非阻塞: 复检失败只置状态位, 不丢弃真实物料(PA12 为权威到达源)。 */
        if (s_sort_busy && (raw_reflect > s_sort_recheck_thresh))
            s_recheck_ok = 1;
    }

    ms_regs[R_F103_ACT_PWM] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);

    /* 网关静默独立检测: 超过 GW_SILENCE_MS 未收到主站合法请求帧 → 网关掉线。
     * 不依赖 F407 通知, 自行进入安全态(停电机), 并标记 GW_DOWN 状态位。
     * F407 恢复轮询后时间戳刷新, 安全态自动解除(电机需重新下发启动指令才转)。 */
    uint8_t gw_down = ((now - s_last_master_tick) >= GW_SILENCE_MS);
    if (gw_down && s_running)
    {
        s_running = 0;
        APP_MOTOR_CTRL_OpenLoop(0, 0);   /* 刹车停电机 */
    }

    /* 自动分拣状态机: 物料经 PA12 对射到达 (NotifyMatArrived 置 s_arrive + 分类 slot) →
       立即打舵(A=1/B=2), 保持 SORT_HOLD_MS 让物料滑入料道 → 回中并计数。 */
    if (s_arrive && !s_sort_busy)
    {
        s_arrive = 0;
        s_sort_busy = 1;
        s_sort_tick = now;
        s_recheck_ok = 0;          /* 新一次分拣, 复检结果清零 */
        BSP_SERVO_Sort(s_arrive_slot);
    }
    /* PB1 反射红外 物料复检: 分拣窗口内按 ADC 扫描节拍采样(见上方 ADC 块),
       物料经过使反射值越线 → 复检通过。非阻塞: 复检失败只置状态位, 不丢弃真实物料。 */
    else if (s_sort_busy && (now - s_sort_tick) >= SORT_HOLD_MS)
    {
        BSP_SERVO_Sort(0);                       /* 回中, 让下一物料通过 */
        MODBUS_SLAVE_NotifySortDone(s_arrive_slot);
        s_recheck_last = s_recheck_ok ? BIT_RECHECK_OK : BIT_RECHECK_FAIL;
        s_recheck_ok = 0;
        s_sort_busy = 0;
    }

    uint16_t status = 0;
    if (s_running) status |= BIT_RUN;
    if (s_fault)   status |= BIT_FAULT;
    if (s_sorting || s_sort_busy) status |= BIT_SORTING;
    if (gw_down)   status |= BIT_GW_DOWN;
    if (s_recheck_last) status |= s_recheck_last;   /* PB1 复检结果状态位(持续显示到下次分拣) */
    ms_regs[R_F103_STATUS] = status;

    /* POST 自检结果汇入故障位: 开机自检未全部通过 → 置 FAULT_POST,
       使 OLED/PC 经 R_F103_FAULT(R_GW_MOTOR_FAULT) 直接可见, 无需额外接线。 */
    if (s_post_mask != POST_ALL)
        s_fault |= FAULT_POST;
    else
        s_fault &= ~FAULT_POST;

    ms_regs[R_F103_FAULT]   = (uint16_t)s_fault;
    ms_regs[R_F103_POST]    = s_post_mask;          /* POST 上电自检结果位掩码 */

    /* 业务计数接入:
     *   MAT_CNT   ← PA12 对射红外 EXTI 下降沿累加 (HAL_GPIO_EXTI_Callback → NotifyMatArrived)
     *   SENSE_A   ← PB1 反射红外 ADC (Rank3, IN9)
     *   SENSE_B   ← PB0 光敏电阻 ADC (Rank2, IN8)
     *   SORT_A/B  ← 舵机回调分拣完成后 NotifySortDone(1/2) 累加,F103 内部应用层变量,
     *               由 F407 网关侧寄存器 R_GW_SORT_A_CNT/R_GW_SORT_B_CNT (reg 5/6) 镜像给 PC。
     *               F103 端这边无对应 Modbus 寄存器(只 9 个 R),所以不写 ms_regs。
     */
    ms_regs[R_F103_MAT_CNT] = s_mat_cnt;
    ms_regs[R_F103_SENSE_A] = BSP_ADC_GetRaw(ADC_RANK_REFLECT_IR);  /* PB1 反射红外 */
    ms_regs[R_F103_SENSE_B] = BSP_ADC_GetRaw(ADC_RANK_LIGHT_SEN);    /* PB0 光敏 */
}

/* ==================== USART1 字节入队 (从中断转给 app_main.c) ==================== */
#define APP_UART1_RX_RING_SIZE   64U
static uint8_t  s_app_uart1_ring[APP_UART1_RX_RING_SIZE];
static volatile uint16_t s_app_uart1_head = 0;
static volatile uint16_t s_app_uart1_tail = 0;

/* 由本文件的 HAL_UART_RxCpltCallback 调用 (中断上下文)。
 * app_main.c 的主循环通过 APP_UART1_GetByte 取字节。
 * 这里只入队,不解析。 */
void APP_UART1_RxByte(uint8_t b)
{
    uint16_t next = (s_app_uart1_head + 1U) % APP_UART1_RX_RING_SIZE;
    if (next != s_app_uart1_tail)        /* 缓冲未满才写 */
    {
        s_app_uart1_ring[s_app_uart1_head] = b;
        s_app_uart1_head = next;
    }
}

/* 主循环出队 — 非中断上下文，返回 1 表示取到一个字节，0 表示缓冲空 */
int APP_UART1_GetByte(uint8_t *b)
{
    if (s_app_uart1_head == s_app_uart1_tail) return 0;
    *b = s_app_uart1_ring[s_app_uart1_tail];
    s_app_uart1_tail = (s_app_uart1_tail + 1U) % APP_UART1_RX_RING_SIZE;
    return 1;
}

/* ==================== GPIO EXTI 强覆盖 (PA12 对射红外) ====================
 * .ioc 配 PA12 = GPIO_MODE_IT_FALLING + 内部上拉
 * stm32f1xx_it.c 的 EXTI15_10_IRQHandler 已调 HAL_GPIO_EXTI_IRQHandler(KEY_INT_Pin)
 * 我们在 HAL_GPIO_EXTI_Callback 判 pin == KEY_INT_Pin (即 PA12) → 物料到达。
 * 默认 HAL 实现是 __weak 空函数, 我们 strong override 否则 s_mat_cnt 永远 0。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_INT_Pin)   /* PA12 = 对射红外, 下降沿=物料到位 */
    {
        MODBUS_SLAVE_NotifyMatArrived();
    }
}
