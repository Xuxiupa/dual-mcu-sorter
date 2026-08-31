#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "stm32f1xx_hal.h"

// 电机硬件初始化：方向引脚(PB3/PB4)配置为推挽输出 + 启动TIM1 CH1 PWM(PA8) + 高级定时器主输出使能
void BSP_MOTOR_Init(void);
void BSP_MOTOR_Forward(void);
void BSP_MOTOR_Backward(void);
void BSP_MOTOR_Brake(void);
// pwm范围：0 ~ 999，和CubeMX TIM1 ARR=999 保持一致
void BSP_MOTOR_SetPwm(uint16_t pwm);
// 开环控制：dir=1 正转+设置PWM，dir=0 刹车(PWM=0)
void BSP_MOTOR_OpenLoop(uint16_t pwm, uint8_t dir);

#endif
