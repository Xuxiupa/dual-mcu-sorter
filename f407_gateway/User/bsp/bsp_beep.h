#ifndef BSP_BEEP_H
#define BSP_BEEP_H

#include <stdint.h>

/* 有源蜂鸣器 (PF8 = TIM13_CH1 PWM, CubeMX 已配 PSC=83/ARR=499 → 2kHz) */
void BEEP_Init(void);
void BEEP_Set(uint8_t on);   /* 1=响(50%占空比 2kHz), 0=静音 */

#endif /* BSP_BEEP_H */
