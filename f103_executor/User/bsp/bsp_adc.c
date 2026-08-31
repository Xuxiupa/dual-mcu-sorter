#include "bsp_adc.h"

extern ADC_HandleTypeDef   hadc1;
extern DMA_HandleTypeDef   hdma_adc1;   /* CubeMX 加 ADC1→DMA1_Ch1 后生成 */

/* ============================================================================
 * ADC + DMA 连续扫描 (替代原 ConfigChannel+Start/Poll/Stop per-rank 补丁)
 *
 * 架构:
 *   - DMA circular 模式: 每次 3 通道扫描完成, DMA 自动把结果写入
 *     s_adc_dma_buf[0..2], 然后立刻重头开始下一轮, 不需要 CPU 介入。
 *   - CPU 任何时候读 s_adc_dma_buf[r] 拿到的都是硬件最新一次的结果
 *     (16-bit 半字读, Cortex-M 原子, 不需要关中断)。
 *   - 取代 BSP_ADC_StartOnce 之前每次循环 "Stop → ConfigChannel 3 次 →
 *     Start/Poll/Stop" 的软件补丁 (8-27 三路同值 bug 修复方案)。
 *
 * CubeMX 必须配:
 *   ADC1:
 *     ScanConvMode          = ENABLE
 *     ContinuousConvMode    = ENABLE       ← 关键: 不用软件触发
 *     DiscontinuousConvMode = DISABLE
 *     NbrOfConversion       = 3
 *     Rank 1: ADC_CHANNEL_4 (PA4 热敏)
 *     Rank 2: ADC_CHANNEL_8 (PB0 光敏)
 *     Rank 3: ADC_CHANNEL_9 (PB1 反射红外)
 *   DMA1 Channel1 (ADC1):
 *     Mode                  = Circular     ← 关键: 自动循环
 *     Data Width            = Half Word
 *     Memory Increment      = Enable
 *     Peripheral Increment  = Disable
 *   NVIC: DMA1_Channel1 global IRQ 勾上 (HAL 默认开启, 不需要写中断服务)
 * ==========================================================================*/

/* ---------------- DMA 目标缓冲 (半字对齐, 数组名即首地址) ---------------- */
static uint16_t s_adc_dma_buf[ADC_CH_TOTAL];

/* ---------------- 滑动平均滤波 (与原版语义一致) ---------------- */
static uint16_t s_filter_buf[ADC_CH_TOTAL][ADC_FILTER_WINDOW];
static uint8_t  s_filter_idx[ADC_CH_TOTAL];   /* 每路独立写位 (原单 s_filter_idx
                                                  在 DMA 任意顺序读下会错位) */
static uint32_t s_filter_sum[ADC_CH_TOTAL];

/* ----------------------------------------------------------------
 * 初始化: 预填滤波窗口 + 一次性启动 DMA circular
 * ----------------------------------------------------------------*/
void BSP_ADC_Init(void)
{
    uint8_t i, w;
    for (i = 0; i < ADC_CH_TOTAL; i++)
    {
        s_filter_idx[i] = 0;
        s_filter_sum[i] = 0;
        s_adc_dma_buf[i] = 0;
        for (w = 0; w < ADC_FILTER_WINDOW; w++)
        {
            s_filter_buf[i][w] = 2048;
            s_filter_sum[i]   += 2048;
        }
    }
    /* 一次性启动, DMA circular 自动维持持续转换, 不再需要每轮 Start/Stop */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma_buf, ADC_CH_TOTAL);
}

/* ----------------------------------------------------------------
 * BSP_ADC_StartOnce 保留为 no-op (兼容老 caller: modbus_slave.c 50ms 节流
 * 调一次, app_main.c POST 自检也调, 都没问题)。
 * DMA 已在 BSP_ADC_Init 启动并持续刷新 s_adc_dma_buf, 这里什么都不做。
 * ----------------------------------------------------------------*/
void BSP_ADC_StartOnce(void)
{
    /* no-op: DMA circular mode 已在 Init 启动, 持续刷新 s_adc_dma_buf */
}

/* 读 → 顺手把最新值塞进滑动窗口, 返回窗口均值 */
static inline void filter_update(uint8_t r, uint16_t raw)
{
    s_filter_sum[r] -= s_filter_buf[r][s_filter_idx[r]];
    s_filter_buf[r][s_filter_idx[r]] = raw;
    s_filter_sum[r] += raw;
    if (++s_filter_idx[r] >= ADC_FILTER_WINDOW) s_filter_idx[r] = 0;
}

uint16_t BSP_ADC_GetRaw(uint8_t rank)
{
    if (rank >= ADC_CH_TOTAL) return 0;
    uint16_t raw = s_adc_dma_buf[rank];   /* 16-bit 原子读, DMA 写半字原子 */
    filter_update(rank, raw);
    return (uint16_t)(s_filter_sum[rank] / ADC_FILTER_WINDOW);
}

uint16_t BSP_ADC_GetRawNoFilter(uint8_t rank)
{
    if (rank >= ADC_CH_TOTAL) return 0;
    return s_adc_dma_buf[rank];
}

int32_t BSP_ADC_GetPotentiometer(int32_t max_out)
{
    uint16_t raw = BSP_ADC_GetRaw(ADC_RANK_POTENTIOMETER);
    if (raw < 20)   return 0;
    if (raw > 4070) return max_out;
    return ((int32_t)raw * max_out) / 4095;
}
