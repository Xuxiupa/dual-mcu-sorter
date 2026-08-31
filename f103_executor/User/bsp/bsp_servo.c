#include "bsp_servo.h"
#include "main.h"
#include "tim.h"      /* htim3, TIM_CHANNEL_1 */

/* 角度→脉冲: SG90 0.5ms(0°)~2.5ms(180°), 周期 20ms(20000 tick @1us)
   pulse = 500 + angle*2000/180  (范围 500..2500) */
static uint16_t angle_to_pulse(uint8_t angle)
{
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    uint32_t p = 500U + (uint32_t)angle * 2000U / 180U;
    return (uint16_t)p;
}

void BSP_SERVO_Init(void)
{
    /* 启动 TIM3 CH1 PWM 输出 (PA6)。htim3 由 CubeMX 生成的 MX_TIM3_Init 配置,
       此处不再软件改周期 (外设配置走 CubeMX)。 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    BSP_SERVO_SetAngle(SERVO_POS_MID);   /* 上电回中位, 避免上电瞬间抖到未知角 */
}

void BSP_SERVO_SetAngle(uint8_t angle)
{
    /* 限位 clamp: 任何来源(默认/分拣/调试)的角度都强制落在 [0,180] */
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, angle_to_pulse(angle));
}

void BSP_SERVO_Sort(uint8_t cmd)
{
    switch (cmd)
    {
        case 1:  BSP_SERVO_SetAngle(SERVO_POS_SORT_A); break;  /* A 料道 */
        case 2:  BSP_SERVO_SetAngle(SERVO_POS_SORT_B); break;  /* B 料道 */
        default: BSP_SERVO_SetAngle(SERVO_POS_MID);    break;  /* 0/非法 → 回中 */
    }
}
