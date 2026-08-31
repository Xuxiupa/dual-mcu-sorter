#include "pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float kf,
              float out_min, float out_max, float int_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Kf = kf;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->int_max = int_max;

    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;

    // 位置式清零
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->integral = 0.0f;
    pid->pos_output = 0.0f;

    // 增量式清零
    pid->e0 = 0.0f;
    pid->e1 = 0.0f;
    pid->e2 = 0.0f;
    pid->inc_output = 0.0f;
}

/* ================================================================
 * 位置式 PID：u(k) = Kp*e + Ki*Σe + Kd*(e(k)-e(k-1)) + Kf*target
 * 输出直接是"总输出"，适合慢速过程/非换向场景
 * ================================================================*/
float PID_Calc_Pos(PID_TypeDef *pid, float target, float feedback)
{
    pid->setpoint = target;
    pid->feedback = feedback;

    pid->err = target - feedback;
    pid->integral += pid->err;

    // 积分限幅（防饱和）
    if (pid->integral >  pid->int_max) pid->integral =  pid->int_max;
    if (pid->integral < -pid->int_max) pid->integral = -pid->int_max;

    pid->pos_output = pid->Kp * pid->err
                    + pid->Ki * pid->integral
                    + pid->Kd * (pid->err - pid->err_last)
                    + pid->Kf * target;   // 目标前馈

    // 输出限幅（-999 ~ +999，方向用正负号区分）
    if (pid->pos_output > pid->out_max) pid->pos_output = pid->out_max;
    if (pid->pos_output < pid->out_min) pid->pos_output = pid->out_min;

    pid->err_last = pid->err;
    return pid->pos_output;
}

/* ================================================================
 * 增量式 PID：Δu(k) = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
 *   核心：每次只算"本次该加多少PWM"，累加到上次输出
 *   优点：
 *     ① 切换方向平滑，无冲击
 *     ② 手动切方向/停机时积分自动被钳制，抗饱和强
 *     ③ 配合前馈项，电机目标速度响应更快
 *   推荐：直流电机传送带调速
 * ================================================================*/
float PID_Calc_Inc(PID_TypeDef *pid, float target, float feedback)
{
    float delta;

    pid->setpoint = target;
    pid->feedback = feedback;

    // 移位：e(k-2) ← e(k-1) ← e(k) ← new_error
    pid->e2 = pid->e1;
    pid->e1 = pid->e0;
    pid->e0 = target - feedback;

    // 标准增量式三项相加（得到"本次需要增加的PWM"）
    delta = pid->Kp * (pid->e0 - pid->e1)
          + pid->Ki * pid->e0
          + pid->Kd * (pid->e0 - 2.0f * pid->e1 + pid->e2);

    // 累加到总输出
    pid->inc_output += delta;

    // 总输出限幅（-999~+999，正负号表示方向，绝对值表示PWM占空比）
    if (pid->inc_output > pid->out_max) pid->inc_output = pid->out_max;
    if (pid->inc_output < pid->out_min) pid->inc_output = pid->out_min;

    return pid->inc_output;
}
