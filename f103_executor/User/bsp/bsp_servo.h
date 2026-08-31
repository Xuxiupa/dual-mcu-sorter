#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

/* ================================================================
 * 舵机驱动 (SG90, TIM3 CH1, PA6)
 * ----------------------------------------------------------------
 * 硬件: SG90 信号线接 PA6 (CubeMX: TIM3_CH1, PWM Generation CH1)
 * 时序: 周期 20ms(50Hz), 脉冲 0.5ms(0°) ~ 2.5ms(180°)
 *   └─ 本工程 TIM3 须配 Prescaler=71 + Period=19999 (1us/tick × 20000 = 20ms)
 *      .ioc 当前 Period=1999(2ms) 是错的, 须用户在 CubeMX 改 (外设配置走 CubeMX 铁律)
 *
 * 限位说明: 舵机机械行程仅 [0°,180°], 越界会堵转烧舵机。
 *   所有摆位入口都先 clamp 到 [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE],
 *   这是舵机限位保护的核心。
 * ================================================================*/

#define SERVO_MIN_ANGLE  0     /* 物理下限, 不可越 */
#define SERVO_MAX_ANGLE  180   /* 物理上限, 不可越 */

/* 三档预设位 (依据分拣机构几何定, 可按实物微调) */
#define SERVO_POS_MID     90   /* 中位: 待机/回中 */
#define SERVO_POS_SORT_A  30   /* A 料道 */
#define SERVO_POS_SORT_B  150  /* B 料道 */

void     BSP_SERVO_Init(void);            /* 启动 PWM + 上电回中位 */
void     BSP_SERVO_SetAngle(uint8_t angle);/* 设定角度 0~180, 越界 clamp */
void     BSP_SERVO_Sort(uint8_t cmd);      /* 0=回中 1=A料道 2=B料道 (接 SORT_CMD) */

#endif /* BSP_SERVO_H */
