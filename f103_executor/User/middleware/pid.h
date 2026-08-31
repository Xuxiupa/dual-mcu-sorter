#ifndef __PID_H
#define __PID_H

/* ================================================================
 * 同时支持位置式 & 增量式两种PID
 *   位置式：简单，适合液位/温度等慢过程
 *   增量式：抗积分饱和 + 切换方向平滑 → 推荐电机调速用
 *   两者都支持前馈Kf（目标越大基础输出越高，加速响应）
 * ================================================================*/
typedef struct
{
    // 增益参数
    float Kp;         // 比例增益
    float Ki;         // 积分增益
    float Kd;         // 微分增益
    float Kf;         // 前馈增益（工业常用，目标前馈补偿）

    float setpoint;   // 目标值
    float feedback;   // 反馈值

    // ---- 位置式PID字段 ----
    float err;
    float err_last;
    float integral;
    float pos_output;

    // ---- 增量式PID字段 ----
    float e0;         // 本次误差 e(k)
    float e1;         // 上次误差 e(k-1)
    float e2;         // 上上次误差 e(k-2)
    float inc_output; // 增量式累加输出

    // ---- 通用限幅 ----
    float out_min;    // 输出下限（电机一般 -999）
    float out_max;    // 输出上限（电机一般 +999）
    float int_max;    // 积分限幅，抗积分饱和
} PID_TypeDef;

/**
 * @brief PID初始化（同时设置两种PID的初始状态）
 * @param kp,ki,kd,kf  四个增益
 * @param out_min/out_max  输出范围（-999~+999 → 正负表方向，绝对值表PWM）
 * @param int_max  积分限幅（≈ out_max * 0.5 ~ 1.0）
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float kf,
              float out_min, float out_max, float int_max);

// 位置式PID计算（保留兼容旧接口）
float PID_Calc_Pos(PID_TypeDef *pid, float target, float feedback);

// 增量式PID计算 + 目标前馈补偿 → 推荐直流电机用
float PID_Calc_Inc(PID_TypeDef *pid, float target, float feedback);

#endif
