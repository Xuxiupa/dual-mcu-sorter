#ifndef __APP_MOTOR_CTRL_H
#define __APP_MOTOR_CTRL_H

#include "pid.h"
#include <stdint.h>

extern PID_TypeDef g_motor_pid;

/* ---------------- 初始化 ---------------- */
// 完整初始化：BSP_MOTOR_Init + BSP_ADC_Init + PID参数 + 刹车待命
void APP_MOTOR_CTRL_Init(void);

/* ---------------- 闭环调速主函数 ---------------- */
// 【必须每10ms调用一次！】配合 Ki 系数；
// 流程：读编码器 → 电位器映射目标转速 → 增量式PID → 方向+PWM输出 → 失速计数
void APP_MOTOR_CTRL_Run(void);

/* ---------------- 外部设置接口 ---------------- */
// 当 "s_use_potentiometer = 0" 时，用这个函数下发目标转速
void APP_MOTOR_CTRL_SetTargetRpm(float rpm);

/* ---------------- 开环接口（自检/调试/强制输出用） ---------------- */
// pwm: 0~999； dir: >0正转 / <0反转 / =0刹车（带短路制动）
void APP_MOTOR_CTRL_OpenLoop(uint16_t pwm, int16_t dir);

/* ---------------- 状态读取接口 ---------------- */
float APP_MOTOR_CTRL_GetTargetRpm(void);  // 当前目标转速
float APP_MOTOR_CTRL_GetRealRpm(void);    // 编码器测得真实转速
float APP_MOTOR_CTRL_GetPidOut(void);     // PID原始输出（-999~+999）
uint16_t APP_MOTOR_CTRL_GetCurPwm(void);   // 实际输出 PWM（开环/闭环统一，供打印与上报）

/* ---------------- 故障接口 ---------------- */
// 失速判定：PWM输出已大于启动阈值 && 真实转速≈0 && 持续超过2秒 → 返回1
uint8_t APP_MOTOR_CTRL_IsStall(void);

// 清除故障状态（复位后/故障解除后调用）：PID积分清零 + 输出归零 + 刹车
void    APP_MOTOR_CTRL_ClearFault(void);

#endif
