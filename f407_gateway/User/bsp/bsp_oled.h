#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>

/* 0.96" SSD1306, 128x64, I2C addr 0x3C (7-bit)
 * 移植自江协科技 4针 I2C 版驱动 (仅底层 I2C 改为 HAL, 其余原样)
 * 定位用 行(Line 1~4) / 列(Column 1~16), 每字 8x16 像素, 直接写屏。 */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif /* BSP_OLED_H */
