#ifndef BSP_W25Q64_H
#define BSP_W25Q64_H

#include <stdint.h>

/* ================================================================
 * W25Q64 软件 SPI 驱动 (bit-bang, 不走硬件 SPI1)
 * ----------------------------------------------------------------
 * 接线: 用户把 W25Q64 接到 TFTLCD 排针的触摸 SPI 脚
 *   SCK  = PB0   (TFTLCD T_SCK)
 *   MISO = PB2   (TFTLCD T_MISO)
 *   MOSI = PF11  (TFTLCD T_MOSI)
 *   CS   = PC13  (TFTLCD T_CS,   备份域引脚, 需 PWR 备份域访问)
 *
 * 为什么不用硬件 SPI1(PA5/6/7):
 *   TFTLCD 排针的 SPI 信号接的是板载 XPT2046 触摸芯片的软件 SPI 脚,
 *   不是 MCU 的硬件 SPI1。硬件 SPI1 在 PA5/6/7 上输出, 而 W25Q64 实际
 *   挂在 PB0/PB2/PF11 上, 两者不在一条总线 → 读回 0xFF (F:FFFF)。
 *   故本驱动用软件模拟 SPI, 不改任何接线。
 *
 * 若你的原理图脚号不同, 只改下面 4 个宏即可, 其余代码不动。
 *
 * 受保护存储: W25Q64_SaveSafe/LoadSafe 在块头加 magic+CRC32, 防止 Flash
 *   位翻转/掉电半写产生静默脏数据(读出校验失败返回损坏, 调用方回退默认)。
 *
 * 命令: 0x9F 读 JEDEC ID | 0x06 写使能 | 0x05 读状态(WIP)
 *       0x20 扇区擦除 | 0x02 页编程(≤256B) | 0x03 读数据
 * W25Q64 必须 SPI Mode3 (CPOL=High, CPHA=2Edge)
 * ================================================================ */

#define W25_SCK_PORT  GPIOB
#define W25_SCK_PIN   GPIO_PIN_0
#define W25_MISO_PORT GPIOB
#define W25_MISO_PIN  GPIO_PIN_2
#define W25_MOSI_PORT GPIOF
#define W25_MOSI_PIN  GPIO_PIN_11
#define W25_CS_PORT   GPIOC
#define W25_CS_PIN    GPIO_PIN_13

void     W25Q64_Init(void);
uint16_t W25Q64_ReadID(void);                    /* 返回 JEDEC ID, 期望 0xEF17 */
void     W25Q64_EraseSector(uint32_t addr);      /* 扇区擦除 4KB */
void     W25Q64_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len); /* ≤256 */
void     W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint16_t len);

/* 受保护存储(防数据丢失): 块内带 magic + CRC32 校验 */
void     W25Q64_SaveSafe(uint32_t addr, const uint8_t *data, uint16_t len);
uint8_t  W25Q64_LoadSafe(uint32_t addr, uint8_t *data, uint16_t maxlen, uint16_t *outlen);
                                 /* 返回 0=成功(已拷贝) 1=损坏/未初始化 */

#endif /* BSP_W25Q64_H */
