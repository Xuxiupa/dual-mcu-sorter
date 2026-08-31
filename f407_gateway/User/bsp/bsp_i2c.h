#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f4xx_hal.h"

/* I2C1 由 CubeMX 配置生成 (PB8=SCL, PB9=SDA, P3 排母 IIC_SCL/SDA)
 *   - hi2c1 / MX_I2C1_Init() 由 CubeMX 生成, 本文件只 extern 引用
 *   - 驱动 OLED (SSD1306, addr 0x3C)
 * 注意: 工程 HAL Driver 原本没装 I2C 模块, 需在 CubeMX 勾 I2C1 重新生成代码,
 *        CubeMX 会自动补 stm32f4xx_hal_i2c.c + 生成 USE_HAL_I2C_MODULE */
extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef BSP_I2C1_Write(uint16_t dev_addr_7bit, uint8_t *pdata, uint16_t len);

#endif /* BSP_I2C_H */
