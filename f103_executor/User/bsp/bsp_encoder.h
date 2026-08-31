#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "stm32f1xx_hal.h"

/* 编码器每转脉冲数(PPR / 线数)。重要：
 *   必须改成你电机上实际编码器的标称值！
 *   - 转速公式 rpm = 脉冲增量 × 60 ÷ (ENCODER_PPR × 4 × dt)
 *   - 其中 ×4 是四倍频(正交编码 A/B 两相各 2 沿)；
 *     若你的编码器规格书写的是 "CPR=xxx"(每转计数)，则 ENCODER_PPR = CPR/4。
 *   - 原值 20 是按 EC11 旋钮写的，电机编码器请按实物改。 */
#define ENCODER_PPR     20U

//启动编码器定时器
void BSP_ENCODER_Init(void);
//获取本次采样脉冲增量，已经做符号取反，解决之前正负方向问题
int16_t BSP_ENCODER_GetDelta(void);
//根据脉冲计算转速 rpm
float BSP_ENCODER_GetRpm(int16_t pulse_cnt);

#endif
