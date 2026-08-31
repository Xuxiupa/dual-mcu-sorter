/* ================================================================
 * F407 板载 I2C1 封装 (HAL 版)
 *   PB8 = I2C1_SCL  (P3 排母 IIC_SCL)
 *   PB9 = I2C1_SDA  (P3 排母 IIC_SDA)
 *
 * 配置责任归 CubeMX: 用户在 CubeMX 勾选 I2C1 + PB8/PB9 配成
 * I2C1_SCL/SDA (开漏上拉, 100/400kHz, 7bit) 重新生成代码后,
 * hi2c1 / MX_I2C1_Init() 由 CubeMX 生成, 这里只 extern 引用。
 * ================================================================ */

#include "bsp_i2c.h"

HAL_StatusTypeDef BSP_I2C1_Write(uint16_t dev_addr_7bit, uint8_t *pdata, uint16_t len)
{
    /* 超时 5ms: 正常 I²C 传输 μs 级, 5ms 足够;
     * 若 OLED 通信异常, 快速失败, 避免 OLED_Clear 1024 次 × 100ms ≈ 104s
     * 把整个系统卡在初始化阶段 (FreeRTOS 调度器起不来)。 */
    return HAL_I2C_Master_Transmit(&hi2c1, dev_addr_7bit << 1, pdata, len, 5);
}
