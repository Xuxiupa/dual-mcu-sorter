#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f1xx_hal.h"

/* ================================================================
 *  ADC通道映射表（必须与CubeMX中ADC1的Rank顺序严格一致！）
 *
 *  请在CubeMX -> ADC1 中重新配置如下：
 *   Number Of Conversion = 4
 *   Rank 1  -> ADC_CHANNEL_4   (PA4)  = 电位器(速度调节)
 *   Rank 2  -> ADC_CHANNEL_8   (PB0)  = 光敏电阻（物料透光/环境光）
 *   Rank 3  -> ADC_CHANNEL_9   (PB1)  = 反射红外（物料颜色/有无）
 *   Rank 4  -> ADC_CHANNEL_TEMPSENSOR (=16, 内部)
 *                                    = MCU内部温度传感器（超温保护）
 *   当前 .ioc 配了 3 路 ADC (IN4/IN8/IN9); IN16 (MCU 温度传感器) 未启用。
 *      ADC_CH_TOTAL=3，ADC_RANK_MCU_TEMP 暂注释。
 *
 *  注意：
 *   - 若启用 MCU 温度传感器, 需在 CubeMX 打勾使能 Temperature Sensor / VrefInt
 *   - Discontinuous Mode 可开可不开，代码两种都兼容
 * ================================================================*/
#define ADC_RANK_POTENTIOMETER   0   // Rank1: PA4-IN4  电位器 → 目标转速
#define ADC_RANK_LIGHT_SEN       1   // Rank2: PB0-IN8  光敏电阻
#define ADC_RANK_REFLECT_IR      2   // Rank3: PB1-IN9  反射红外
// #define ADC_RANK_MCU_TEMP        3   // Rank4: IN16  启用 IN16 后再开

#define ADC_CH_TOTAL             3   // 实际配 3 路, 与 CubeMX NbrOfConversion 一致

// 滑动平均滤波窗口大小（越大越稳定，但是反应越慢）
#define ADC_FILTER_WINDOW        8

/* ---------------- API ---------------- */

// 初始化ADC缓冲（预填窗口避免上电异常值）
void BSP_ADC_Init(void);

// 触发一次所有通道转换 + 更新滤波器；主循环每隔1~10ms调用一次
void BSP_ADC_StartOnce(void);

// 获取某一路的滤波后原始值（0~4095）
uint16_t BSP_ADC_GetRaw(uint8_t rank);

// 获取某一路未经滤波的本次采样值（调试用）
uint16_t BSP_ADC_GetRawNoFilter(uint8_t rank);

// 电位器AD值 → 线性映射 0~max_out；死区处理
int32_t BSP_ADC_GetPotentiometer(int32_t max_out);


#endif
