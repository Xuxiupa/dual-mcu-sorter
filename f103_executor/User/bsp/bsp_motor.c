#include "bsp_motor.h"
#include "main.h"

/* ================================================================
 * 根据官方引脚配置表：
 *   TB6612 AIN1  →  PB3  (GPIO Output)
 *   TB6612 AIN2  →  PB4  (GPIO Output)
 *   TB6612 PWMA  →  PA8  (TIM1_CH1 PWM)
 *   编码器 A/B   →  PA0/PA1 (TIM2, 不冲突)
 *   注意：PA2/PA3是USART2（双MCU通信用），绝对不能占用！
 * ================================================================*/
#define AIN1_GPIO_PORT  GPIOB
#define AIN1_GPIO_PIN   GPIO_PIN_3
#define AIN2_GPIO_PORT  GPIOB
#define AIN2_GPIO_PIN   GPIO_PIN_4

extern TIM_HandleTypeDef htim1;   // PWM定时器句柄（tim.c中定义）

/* ----------------------------------------------------------------
 * 电机硬件初始化
 *   - 配置PB3/PB4为推挽高速输出（AIN1/AIN2）
 *   - 上电默认刹车(0,0)，避免乱转
 *   - 启动TIM1 CH1 PWM输出（引脚PA8输出PWM波给PWMA）
 *   - TIM1是高级定时器，必须开启主输出MOE，否则PWM引脚始终无输出！
 * ----------------------------------------------------------------*/
void BSP_MOTOR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ① 使能GPIOB时钟（通常gpio.c已开，保险起见再开一次）
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // ② 配置PB3/PB4为推挽高速输出
    GPIO_InitStruct.Pin   = AIN1_GPIO_PIN | AIN2_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AIN1_GPIO_PORT, &GPIO_InitStruct);

    // ③ 默认刹车，避免上电瞬间乱转
    BSP_MOTOR_Brake();

    // ④ 启动TIM1通道1的PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    // ⑤ ★高级定时器(TIM1/TIM8)特有：必须手动开启主输出MOE
    //    这一步不做，PA8引脚永远是低电平，PWM完全没输出！
    __HAL_TIM_MOE_ENABLE(&htim1);
}

/* ----------------------------------------------------------------
 * 正转：AIN1=1, AIN2=0
 * ----------------------------------------------------------------*/
void BSP_MOTOR_Forward(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_PORT, AIN1_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_PORT, AIN2_GPIO_PIN, GPIO_PIN_RESET);
}

/* ----------------------------------------------------------------
 * 反转：AIN1=0, AIN2=1
 * ----------------------------------------------------------------*/
void BSP_MOTOR_Backward(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_PORT, AIN1_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_PORT, AIN2_GPIO_PIN, GPIO_PIN_SET);
}

/* ----------------------------------------------------------------
 * 刹车（短路制动）：AIN1=0, AIN2=0
 *   比单纯PWM=0更有制动力，工业常用
 * ----------------------------------------------------------------*/
void BSP_MOTOR_Brake(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_PORT, AIN1_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_PORT, AIN2_GPIO_PIN, GPIO_PIN_RESET);
}

/* ----------------------------------------------------------------
 * 设置PWM占空比（0~999 → 0%~100%）
 * ----------------------------------------------------------------*/
void BSP_MOTOR_SetPwm(uint16_t pwm)
{
    if(pwm > 999) pwm = 999;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
}

/* 开环控制：dir=1 正转+设置PWM, dir=0 刹车(PWM=0)
 *   用于无编码器场景下的简易调速（旋钮或串口命令） */
void BSP_MOTOR_OpenLoop(uint16_t pwm, uint8_t dir)
{
    if (dir) {
        BSP_MOTOR_Forward();
    } else {
        BSP_MOTOR_Brake();
        pwm = 0;
    }
    BSP_MOTOR_SetPwm(pwm);
}
