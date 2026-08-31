/* ================================================================
 * 板载有源蜂鸣器驱动 (PF8 = TIM13_CH1)
 *   CubeMX 已配: TIM13 PWM Generation CH1, PSC=83, ARR=499 → 2kHz
 *   MX_TIM13_Init() 由 main.c 外设初始化调用, 初始 Pulse=0(静音)。
 *   本文件只做: 启动 PWM + 用 CCR 控制响/停 (有源蜂鸣器, 高电平即响)。
 * ================================================================ */

#include "bsp_beep.h"
#include "tim.h"       /* htim13 (CubeMX 生成, tim.c) */
#include "main.h"

#define BEEP_ON_CCR    250   /* ARR=499 的 50% → 2kHz 方波 */

void BEEP_Init(void)
{
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);   /* 静音 */
}

void BEEP_Set(uint8_t on)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1,
                          on ? BEEP_ON_CCR : 0);
}
