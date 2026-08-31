#include "app_main.h"
#include "app_motor_ctrl.h"
#include "modbus_slave.h"     /* Modbus 从站 (USART2 + 接管 USART1 RX 中断入口) */
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_adc.h"          /* BSP_ADC_StartOnce / BSP_ADC_GetRaw (POST 自检) */
#include "bsp_servo.h"        /* 舵机限位驱动 */
#include "mb_regmap.h"        /* POST_* 位定义 (POST_ADC 等) */
#include "main.h"
#include "usart.h"
#include "iwdg.h"             /* hiwdg + HAL_IWDG_Refresh (独立看门狗) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * 电机开环 PWM 调速 (电机只有 2 根线, 无轴编码器, PID 闭环不可行)
 *   接线：TB6612 PWMA←PA8, AIN1←PB3, AIN2←PB4, STBY 需接高电平
 *        EC11 旋转编码器 ← PA0/PA1 (TIM2 编码器模式), 用作手动调速旋钮
 *   调速方式（开环, PWM≈转速）：
 *     - 拧 EC11 旋钮：右拧加速、左拧减速到 0 刹车
 *     - 串口命令：O<pwm>/R<pwm> 设定占空比(0~999), S 停止, E 看编码器计数
 *   说明：PB3/PB4 是 JTAG 脚, 已在 main() 中释放(直接写 AFIO->MAPR, 保留 SWD)
 * ================================================================*/

static char s_line[24];
static uint16_t s_knob_pwm = 0;        /* EC11 旋钮对应的目标占空比(0~999) */
static uint8_t s_line_idx = 0;
static uint32_t s_last_ch_tick = 0;   /* 上次收到字符的时间戳，用于超时触发 parse_line */

#define LINE_SILENCE_MS  200U  /* 静默超过 200ms 即认为一行输入结束 */

/* ---- USART1 接收：原 RXNE 中断 + 环形缓冲已移交给 modbus_slave.c。
   app_main.c 只通过 APP_UART1_GetByte() 取字节，不再维护 s_rx_ring。 ---- */
extern TIM_HandleTypeDef htim2;   /* 编码器定时器，tim.c 中定义 */
extern TIM_HandleTypeDef htim1;   /* 电机 PWM 定时器(TIM1 CH1)，bsp_motor.c 中定义 */
extern TIM_HandleTypeDef htim3;   /* 舵机 PWM 定时器(TIM3 CH1)，bsp_servo.c 中定义 */

/* ================================================================
 * POST 上电自检 (系统级安全): 对各外设做最小可用性检查(配置/存在性),
 * 结果按位返回, 由 modbus_slave 经 R_F103_POST(reg9) 上送 F407/PC。
 *   不实际转动电机(无负载/安全), 只验证外设配置与 ADC 转换能力。
 *   位定义见 mb_regmap.h: POST_ADC / POST_MOTOR / POST_ENC / POST_ALL
 * ================================================================*/
static uint16_t post_run(void)
{
    uint16_t m = 0;

    /* 1) ADC: 3 路扫描, 转换成功且非三路全卡在极端值(说明时钟+参考正常,
       非 ADC 总线死锁)。任一路在中间区即认为 ADC 正常。 */
    BSP_ADC_StartOnce();
    uint16_t r0 = BSP_ADC_GetRaw(ADC_RANK_POTENTIOMETER);
    uint16_t r1 = BSP_ADC_GetRaw(ADC_RANK_LIGHT_SEN);
    uint16_t r2 = BSP_ADC_GetRaw(ADC_RANK_REFLECT_IR);
    if (!((r0 == 0 && r1 == 0 && r2 == 0) ||
          (r0 == 4095 && r1 == 4095 && r2 == 4095)))
        m |= POST_ADC;

    /* 2) 电机/TB6612: TIM1 PWM 外设已配置(htim1.Instance 非空即 CubeMX 已生成) */
    if (htim1.Instance != NULL) m |= POST_MOTOR;

    /* 3) 编码器/旋钮: TIM2 编码器模式已配置 */
    if (htim2.Instance != NULL) m |= POST_ENC;

    /* 4) 舵机: TIM3 PWM 外设已配置 (htim3.Instance 非空) */
    if (htim3.Instance != NULL) m |= POST_SERVO;

    return m;
}

static void parse_line(char *line)
{
    /* 诊断：打印字符串 + 每个字节的hex值，定位 line[0] 为什么不匹配已知命令 */
    printf("[DBG] got:'%s' [", line);
    for (int i = 0; line[i] && i < 8; i++) printf("%02X ", (uint8_t)line[i]);
    printf("]\r\n");

    /* 容错：跳过前导非ASCII/不可见字符（比如SSCOM发送框粘来的全角空格 0xA1 0xA1），
       并把"数字 0"也当成字母 O 接受（SSCOM的0和O视觉上极像，调试期间极易踩坑），
       匹配命令首字母。 */
    char *p = line;
    while (*p && (uint8_t)*p < 0x21) p++;   // 跳过控制字符和空格

    /* 电机只有 2 根线无轴编码器, PA0/1 的 EC11 是手动旋钮不能测电机转速,
       故采用开环 PWM, R/O 同义设定占空比. */
    if (*p == 'R' || *p == 'r' || *p == 'O' || *p == 'o' || *p == '0')
    {
        uint16_t pwm = (uint16_t)atoi(p + 1);
        if (pwm > 999) pwm = 999;
        s_knob_pwm = pwm;
        BSP_MOTOR_OpenLoop(pwm, 1);
        printf(">> pwm=%u (open-loop)\r\n", pwm);
    }
    else if (*p == 'S' || *p == 's')
    {
        s_knob_pwm = 0;
        BSP_MOTOR_OpenLoop(0, 0);
        printf(">> stopped\r\n");
    }
    else if (*p == '?')
    {
        printf("O/R<pwm>(0~999) set | S stop | E enc-cnt | V<deg> servo | A/B sort | TL/TR<thr> 标定 | CL 打印 | C3 三路ADC | ? help\r\n");
    }
    else if (*p == 'E' || *p == 'e')
    {
        /* 诊断：手拧编码器时观察 CNT 是否变化，判断编码器接线/供电是否正常 */
        printf("CNT=%d\r\n", (int16_t)__HAL_TIM_GET_COUNTER(&htim2));
    }
    /* ============ 串口直驱舵机调试 (#4): 绕过 PA12 验证舵机本身 ============
       V<角度>  直定角度(0~180, 隔离"供电 vs 信号"问题, 不依赖 PA12 触发)
       A / B     直接打向 A(30°) / B(150°) 料道
       （需 5V 供电, 3.3V 可能转不动） */
    else if (*p == 'V' || *p == 'v')
    {
        uint16_t deg = (uint16_t)atoi(p + 1);
        if (deg > 180) deg = 180;
        BSP_SERVO_SetAngle((uint8_t)deg);
        printf(">> servo angle=%u\r\n", deg);
    }
    else if (*p == 'A' || *p == 'a')
    {
        BSP_SERVO_Sort(1);
        printf(">> sort A (30 deg)\r\n");
    }
    else if (*p == 'B' || *p == 'b')
    {
        BSP_SERVO_Sort(2);
        printf(">> sort B (150 deg)\r\n");
    }
    /* ============ 分拣/复检阈值标定 (#5): 运行期可调, 不改寄存器映射 ============
       CL        打印当前光敏/反射原始值 + 两个阈值(标定参考)
       TL <n>    设定光敏分类阈值(>n→A料道, 否则B)
       TR <n>    设定 PB1 反射红外复检阈值(越线=复检通过) */
    else if (*p == 'C' || *p == 'c')
    {
        if (p[1] == 'L' || p[1] == 'l')
        {
            uint16_t rl = BSP_ADC_GetRaw(ADC_RANK_LIGHT_SEN);
            uint16_t rr = BSP_ADC_GetRaw(ADC_RANK_REFLECT_IR);
            printf("light=%u recheck=%u | thr_light=%u thr_recheck=%u\r\n",
                   rl, rr, MODBUS_SLAVE_GetLightThresh(), MODBUS_SLAVE_GetRecheckThresh());
        }
        else if (p[1] == '3')
        {
            /* C3: 一行打 3 路原始值, 验 ADC+DMA 通道独立性。
             * 期望: 捂 PA4 只 PA4 动, PB0/PB1 不动 → DMA 改对 */
            printf("PA4=%u PB0=%u PB1=%u\r\n",
                   BSP_ADC_GetRaw(ADC_RANK_POTENTIOMETER),
                   BSP_ADC_GetRaw(ADC_RANK_LIGHT_SEN),
                   BSP_ADC_GetRaw(ADC_RANK_REFLECT_IR));
        }
        else
            printf("unknown C cmd\r\n");
    }
    else if (*p == 'T' || *p == 't')
    {
        if (p[1] == 'L' || p[1] == 'l')
        {
            MODBUS_SLAVE_SetLightThresh((uint16_t)atoi(p + 2));
            printf(">> light_thresh=%u\r\n", MODBUS_SLAVE_GetLightThresh());
        }
        else if (p[1] == 'R' || p[1] == 'r')
        {
            MODBUS_SLAVE_SetRecheckThresh((uint16_t)atoi(p + 2));
            printf(">> recheck_thresh=%u\r\n", MODBUS_SLAVE_GetRecheckThresh());
        }
        else
            printf("unknown T cmd\r\n");
    }
    else
    {
        printf("unknown cmd: %s (type ?)\r\n", line);
    }
}

void app_main(void)
{
    /* ★ 看门狗说明: WWDG(窗口型) 已在 MX_WWDG_Init 启动, counter 从 127 递减。
       严禁在 counter 仍 > Window 时立即喂狗(判为"太早"→复位), 故此处不喂,
       等进主循环、counter 降到窗口内再喂。启动期禁止 printf(阻塞串口并占用
       WWDG 窗口), 已移除, 避免 init 期间 counter 掉到窗口下导致上电即复位。 */
    MODBUS_SLAVE_Init();              /* 启动 USART1 + USART2 RXNE 中断（Modbus 从站接管） */
    BSP_ENCODER_Init();
    APP_MOTOR_CTRL_Init();
    BSP_SERVO_Init();                 /* 启动舵机 PWM + 上电回中位 */

    /* POST 上电自检: 检查 ADC/电机/编码器外设可用性, 结果经 R_F103_POST 上送 */
    MODBUS_SLAVE_ReportPost(post_run());

    uint32_t last_print = 0;

    while (1)
    {
        /* ★ 看门狗喂狗: F103 的 WWDG 已 Disable(窗口型太严苛, 易误触发复位),
           仅保留 IWDG(2s 宽松)作主保护。主循环每轮≈10ms 远在窗口内, 安全。
           任一段卡死>2s → IWDG 自动复位。 */
        HAL_IWDG_Refresh(&hiwdg);

        /* EC11 旋钮调速 —— 旋钮接 PA0/PA1 (TIM2 编码器模式).
           电机只有 2 根线无轴编码器, PID 闭环不可行, 退回开环.
           右拧加速、左拧减速到 0 刹车; 不拧则保持当前占空比. */
        int16_t enc_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        static int16_t enc_last = 0;
        int16_t d = enc_now - enc_last;
        enc_last = enc_now;
        if (d != 0)
        {
            int32_t next = (int32_t)s_knob_pwm + (int32_t)d * 10;
            if (next < 0)   next = 0;
            if (next > 999) next = 999;
            s_knob_pwm = (uint16_t)next;
            BSP_MOTOR_OpenLoop(s_knob_pwm, (int16_t)(s_knob_pwm > 0 ? 1 : 0));
            printf("knob->pwm=%u\r\n", s_knob_pwm);
        }

        /* 串口命令解析：从环形缓冲读取所有已到达的字节（中断已收妥，绝不丢）。
           配合下方"静默超时触发"逻辑，SSCOM 勾不勾回车换行都能解析。
           USART1 环形缓冲在 modbus_slave.c 里，这里只通过 APP_UART1_GetByte 出队。 */
        uint8_t ch;
        while (APP_UART1_GetByte(&ch))
        {
            s_last_ch_tick = HAL_GetTick();   /* 更新最后收到字符时间 */
            if (ch == '\r' || ch == '\n')
            {
                if (s_line_idx > 0)
                {
                    s_line[s_line_idx] = '\0';
                    parse_line(s_line);
                    s_line_idx = 0;
                }
            }
            else if (s_line_idx < (sizeof(s_line) - 1))
            {
                s_line[s_line_idx++] = (char)ch;
            }
        }

        /* 超时触发：静默超过 LINE_SILENCE_MS 即把已收字符当作一行解析。
           这样无需串口助手发送 \r\n，R60 / S / ? 都能识别。 */
        if (s_line_idx > 0 && (HAL_GetTick() - s_last_ch_tick) >= LINE_SILENCE_MS)
        {
            s_line[s_line_idx] = '\0';
            parse_line(s_line);
            s_line_idx = 0;
        }

        /* 每 1000ms 打印一次状态: 目标/实际转速 + PID 输出 + PWM */
        if (HAL_GetTick() - last_print >= 1000)
        {
            last_print = HAL_GetTick();
            printf("pwm=%5u  (open-loop, no encoder)\r\n", APP_MOTOR_CTRL_GetCurPwm());
        }

        /* Modbus 从站轮询：处理 PC→F103 请求 + 同步写命令到电机 + 回填状态寄存器。
           必须每 ~10ms 调一次，与主循环 HAL_Delay(10) 节拍一致。 */
        MODBUS_SLAVE_Poll();

        HAL_Delay(10);
    }
}
