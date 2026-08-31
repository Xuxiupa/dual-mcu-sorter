#include "app_motor_ctrl.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"

PID_TypeDef g_motor_pid;

/* ---------- 内部状态 ---------- */
static float s_target_rpm    = 0.0f;
static float s_real_rpm      = 0.0f;
static float s_pid_out       = 0.0f;
static uint32_t s_last_tick  = 0;     // 节拍基准(ms)，用于按实际dt算转速
static uint16_t s_cur_pwm    = 0;     // 实际输出到电机的 PWM(开环/闭环统一记录，供打印与上报)
#define CTRL_PERIOD_MS       50U     // 闭环节流周期(50ms = 20Hz)
                                       // 编码器测速需要足够采样窗口：
                                       // PPR=20 时 60rpm 在 50ms 内约 4 脉冲，可稳定测量；
                                       // 主循环每 3ms 调一次 Run 会导致 GetDelta 频繁清零，
                                       // 短采样窗口内永远采不到 1 个脉冲，act 永远显示 0。

// =1 启用电位器(PA4)直接做目标转速；=0 使用APP_MOTOR_CTRL_SetTargetRpm外部下发
// 面包板未接电位器到PA4(PA4已改为热敏电阻)，目标转速由串口命令设定
static uint8_t  s_use_potentiometer = 0;

#define MAX_TARGET_RPM    180.0f   // 电位器拧满 = 180rpm

/* ---------- 失速检测参数 ---------- */
static uint32_t s_stall_cnt_ms = 0;
#define STALL_PWM_MIN     150     // |PWM| > 这个值，才算"要求电机转起来"
#define STALL_RPM_MAX     3.0f    // |真实转速| < 这个值，才算"疑似没转"
#define STALL_TIME_MS     2000    // 上面两个条件同时持续 2s → 判失速

/* ======================================================================
 * 初始化：顺序 BSP_MOTOR → BSP_ADC → 增量式PID初值 → 刹车待命
 * ======================================================================*/
void APP_MOTOR_CTRL_Init(void)
{
    // ① 电机硬件：方向脚PB3/PB4输出 + 启动TIM1 PWM(PA8) + 开MOE
    BSP_MOTOR_Init();

    // ② ADC: bsp_adc.h ADC_CH_TOTAL=3 对应 .ioc 配的 IN4/IN8/IN9,
    //    内部温度传感器 IN16 暂未配 → BSP_ADC_GetMcuTempC 已 #if 0 关掉)
    BSP_ADC_Init();

    // 记录节拍基准
    s_last_tick = HAL_GetTick();

    // ③ 增量式PID初值（新手友好：先只调P，再慢慢加I和D）
    //    先用温和参数：Kp=1.5  Ki=0.1  Kd=0.0  Kf=0.0
    //    输出限幅 -999~+999，积分限幅 ±200
    PID_Init(&g_motor_pid,
             1.5f, 0.1f, 0.0f, 0.0f,
             -999.0f, +999.0f, 200.0f);

    // ④ 刹车待命
    BSP_MOTOR_Brake();
    APP_MOTOR_CTRL_ClearFault();
}

/* ======================================================================
 * 外部下发目标转速（配合 s_use_potentiometer=0 时使用）
 * ======================================================================*/
void APP_MOTOR_CTRL_SetTargetRpm(float rpm)
{
    if (rpm >  MAX_TARGET_RPM) rpm =  MAX_TARGET_RPM;
    if (rpm < -MAX_TARGET_RPM) rpm = -MAX_TARGET_RPM;
    s_target_rpm = rpm;
}

/* ======================================================================
 * 开环控制（自检/调试/强制输出用，不走PID）
 * ======================================================================*/
void APP_MOTOR_CTRL_OpenLoop(uint16_t pwm, int16_t dir)
{
    if (pwm > 999) pwm = 999;

    if (dir > 0)       { BSP_MOTOR_Forward();  BSP_MOTOR_SetPwm(pwm); s_cur_pwm = pwm; }
    else if (dir < 0)  { BSP_MOTOR_Backward(); BSP_MOTOR_SetPwm(pwm); s_cur_pwm = pwm; }
    else               { BSP_MOTOR_Brake();    BSP_MOTOR_SetPwm(0);   s_cur_pwm = 0;   }
}

/* ======================================================================
 * 【核心】闭环调速 Run()  —— 建议每10ms调用一次（内部按实际 dt 计算转速，
 *                         所以即使调用周期略有抖动也不影响转速精度）
 * ======================================================================*/
void APP_MOTOR_CTRL_Run(void)
{
    // 0) 目标转速=0 时强制刹车 + 清空PID历史，避免PID静态残留输出持续驱动电机
    //    （否则上电抖动/外力带动编码器后，PID累积输出会卡在某个非零值停不下来）
    if (s_target_rpm == 0.0f)
    {
        BSP_MOTOR_Brake();
        BSP_MOTOR_SetPwm(0);
        s_cur_pwm = 0;
        APP_MOTOR_CTRL_ClearFault();
        s_real_rpm = 0.0f;
        s_pid_out  = 0.0f;
        return;
    }

    // 节流：编码器测速需要足够采样窗口，否则短脉冲被舍入到 0
    // 主循环每 ~3ms 调一次 Run，没节流时 GetDelta 每 3ms 把 CNT 清零，
    // 60rpm 时 3ms 内约 0.24 个脉冲 → 永远读不到 1 个完整脉冲 → act 恒为 0
    uint32_t now = HAL_GetTick();
    if (now - s_last_tick < CTRL_PERIOD_MS) return;
    uint32_t elapsed_ms = now - s_last_tick;
    float dt = (float)elapsed_ms / 1000.0f;        // 实际间隔(秒)，约 0.05s
    if (dt <= 0.0f) dt = 0.05f;                    // 兜底
    s_last_tick = now;

    // 1) 读编码器脉冲增量（TIM2 CNT清零方式） → 实际转速(rpm)
    //    rpm = 脉冲增量 × 60 ÷ (每转脉冲数 × dt)
    int16_t delta = BSP_ENCODER_GetDelta();
    s_real_rpm = (float)delta * 60.0f / (ENCODER_PPR * 4.0f * dt);

    // 2) 目标转速来源：由串口命令设定；若启用电位器则此处映射
    if (s_use_potentiometer)
    {
        int32_t mapped_x10 = BSP_ADC_GetPotentiometer((int32_t)(MAX_TARGET_RPM * 10.0f));
        s_target_rpm = (float)mapped_x10 / 10.0f;
    }

    // 3) 增量式PID计算 → 输出范围-999~+999（正负=方向，绝对值=PWM占空比）
    s_pid_out = PID_Calc_Inc(&g_motor_pid, s_target_rpm, s_real_rpm);

    // 4) 方向 + PWM 输出（死区内刹车，避免小PWM电机不动还发热）
    if (s_pid_out > 10.0f)
    {
        BSP_MOTOR_Forward();
        BSP_MOTOR_SetPwm((uint16_t)s_pid_out);
        s_cur_pwm = (uint16_t)s_pid_out;
    }
    else if (s_pid_out < -10.0f)
    {
        BSP_MOTOR_Backward();
        BSP_MOTOR_SetPwm((uint16_t)(-s_pid_out));
        s_cur_pwm = (uint16_t)(-s_pid_out);
    }
    else
    {
        BSP_MOTOR_Brake();
        BSP_MOTOR_SetPwm(0);
        s_cur_pwm = 0;
    }

    // 5) 失速检测（3重条件，缺一不可，避免"用户设定转速=0时误报警"）
    if ((s_pid_out > STALL_PWM_MIN || s_pid_out < -STALL_PWM_MIN)   // ① 确实要求输出
        && (s_real_rpm > -STALL_RPM_MAX && s_real_rpm < STALL_RPM_MAX)) // ② 但电机没转
    {
        s_stall_cnt_ms += elapsed_ms;   // ③ 按实际经过时间累加
    }
    else
    {
        // 慢慢衰减，避免偶发毛刺导致报警/解除
        if (s_stall_cnt_ms > elapsed_ms) s_stall_cnt_ms -= elapsed_ms;
        else s_stall_cnt_ms = 0;
    }
}

/* ======================================================================
 * 读取接口
 * ======================================================================*/
float APP_MOTOR_CTRL_GetTargetRpm(void){ return s_target_rpm; }
float APP_MOTOR_CTRL_GetRealRpm(void)  { return s_real_rpm; }
float APP_MOTOR_CTRL_GetPidOut(void)   { return s_pid_out; }

uint16_t APP_MOTOR_CTRL_GetCurPwm(void) { return s_cur_pwm; }

uint8_t APP_MOTOR_CTRL_IsStall(void)
{
    return (s_stall_cnt_ms >= STALL_TIME_MS) ? 1 : 0;
}

/* ======================================================================
 * 清除故障状态（复位/故障解除后调用）
 * ======================================================================*/
void APP_MOTOR_CTRL_ClearFault(void)
{
    s_stall_cnt_ms = 0;

    // PID全清零，防止重启瞬间有历史输出冲击
    g_motor_pid.inc_output = 0;
    g_motor_pid.pos_output = 0;
    g_motor_pid.e0 = g_motor_pid.e1 = g_motor_pid.e2 = 0;
    g_motor_pid.integral = 0;
    g_motor_pid.err = 0;
    g_motor_pid.err_last = 0;

    s_target_rpm = 0;
    s_real_rpm   = 0;
    s_pid_out    = 0;

    BSP_MOTOR_Brake();
    BSP_MOTOR_SetPwm(0);
    s_cur_pwm = 0;
}
