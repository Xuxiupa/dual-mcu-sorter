/* ================================================================
 * W25Q64 SPI Flash 驱动 (F407, 软件 SPI / bit-bang)
 *   引脚: SCK=PB0, MISO=PB2, MOSI=PF11, CS=PC13 (TFTLCD 排针触摸 SPI 脚)
 *   命令: 0x9F 读 JEDEC ID | 0x06 写使能 | 0x05 读状态(WIP)
 *         0x20 扇区擦除 | 0x02 页编程(≤256B) | 0x03 读数据
 *   W25Q64 必须 SPI Mode3 (CPOL=High, CPHA=2Edge)
 * ================================================================ */

#include "bsp_w25q64.h"
#include "gpio.h"        /* HAL_GPIO_WritePin / ReadPin / Init */
#include "main.h"        /* HAL_PWR_EnableBkUpAccess */
#include <string.h>     /* memcpy */

/* ---------- 底层: CS + 软件 SPI 单字节收发 ---------- */
static void w25_delay(void)
{
    /* 软件 SPI 降速: 循环 32→300, SCK 周期降到几十 us 级 (~几十 kHz),
       给 W25Q64 足够 MISO 数据建立时间, 解决读 ID 偶发丢 bit (0xEF17→0xEF4)。
       168MHz 下 300 次 volatile 空循环约数 us, W25Q64 最高支持 80MHz 完全够用。 */
    for (volatile uint32_t i = 0; i < 300; i++);
}

static void cs_low(void)  { HAL_GPIO_WritePin(W25_CS_PORT,  W25_CS_PIN,  GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(W25_CS_PORT,  W25_CS_PIN,  GPIO_PIN_SET);   }

/* Mode3 (CPOL=1, CPHA=1): SCK 空闲高, 第2边沿(上升沿)采样 */
static uint8_t spi_xfer(uint8_t byte)
{
    uint8_t rx = 0;
    for (int i = 7; i >= 0; i--)
    {
        HAL_GPIO_WritePin(W25_SCK_PORT,  W25_SCK_PIN,  GPIO_PIN_RESET); /* 拉低(下降沿前放 MOSI) */
        HAL_GPIO_WritePin(W25_MOSI_PORT, W25_MOSI_PIN,
                          ((byte >> i) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        w25_delay();
        HAL_GPIO_WritePin(W25_SCK_PORT,  W25_SCK_PIN,  GPIO_PIN_SET);   /* 拉高(上升沿: 从机采样 MOSI) */
        w25_delay();                                                   /* ★ 等从机把 MISO 数据建立稳定 */
        if (HAL_GPIO_ReadPin(W25_MISO_PORT, W25_MISO_PIN) == GPIO_PIN_SET)
            rx = (uint8_t)((rx << 1) | 1);
        else
            rx = (uint8_t)(rx << 1);
    }
    return rx;
}

/* ---------- 内部辅助 ---------- */
static void w25_write_enable(void)
{
    cs_low();
    spi_xfer(0x06);      /* WRITE ENABLE */
    cs_high();
}

static void w25_wait_busy(void)
{
    uint8_t status;
    uint32_t tries = 0;
    cs_low();
    spi_xfer(0x05);      /* READ STATUS REGISTER */
    /* ★ 超时保护: 软件 SPI 面包板时序敏感, 若 W25Q64 偶发不应答(CS 未拉低/位错),
       WIP 永远为 1 → 原 do-while 永久死锁 → APP_MAIN_Init 卡死 → 系统起不来。
       加有界重试: 正常擦扇区 ≤400ms 必定退出; 故障时快速返回, 不拖垮整机。 */
    do {
        status = spi_xfer(0xFF);
        if (++tries > 50000UL) break;   /* ~几百 ms 上限, 足够正常完成 */
    } while (status & 0x01);            /* WIP=忙 */
    cs_high();
}

/* ---------- 对外接口 ---------- */
void W25Q64_Init(void)
{
    /* PC13 是 F407 备份域引脚 (RTC/TAMP): 备份域写保护默认开启时,
       写 BSRR 拉低 PC13 可能被屏蔽 → CS 无效 → 必须先开备份域访问。
       引脚方向 (PB0=SCK / PB2=MISO / PF11=MOSI / PC13=CS) 由 CubeMX
       在 MX_GPIO_Init 配置, 本驱动不再软件自配 GPIO。 */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    HAL_GPIO_WritePin(W25_SCK_PORT, W25_SCK_PIN, GPIO_PIN_SET); /* SCK 空闲高 (Mode3) */
    cs_high();           /* 未选中 */
}

uint16_t W25Q64_ReadID(void)
{
    uint8_t mid, did;
    cs_low();
    spi_xfer(0x9F);      /* JEDEC ID */
    mid = spi_xfer(0xFF);
    did = spi_xfer(0xFF);
    cs_high();
    return ((uint16_t)mid << 8) | did;   /* W25Q64 = 0xEF17 */
}

void W25Q64_EraseSector(uint32_t addr)
{
    w25_write_enable();
    cs_low();
    spi_xfer(0x20);      /* SECTOR ERASE 4KB */
    spi_xfer((uint8_t)(addr >> 16));
    spi_xfer((uint8_t)(addr >> 8));
    spi_xfer((uint8_t)addr);
    cs_high();
    w25_wait_busy();
}

void W25Q64_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    if (len > 256) len = 256;
    w25_write_enable();
    cs_low();
    spi_xfer(0x02);      /* PAGE PROGRAM */
    spi_xfer((uint8_t)(addr >> 16));
    spi_xfer((uint8_t)(addr >> 8));
    spi_xfer((uint8_t)addr);
    for (uint16_t i = 0; i < len; i++) spi_xfer(buf[i]);
    cs_high();
    w25_wait_busy();
}

void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint16_t len)
{
    cs_low();
    spi_xfer(0x03);      /* READ DATA */
    spi_xfer((uint8_t)(addr >> 16));
    spi_xfer((uint8_t)(addr >> 8));
    spi_xfer((uint8_t)addr);
    for (uint16_t i = 0; i < len; i++) buf[i] = spi_xfer(0xFF);
    cs_high();
}

/* ====================== 受保护存储（防数据丢失） ======================
 * 裸 ReadData/WritePage 不带校验, Flash 位翻转或掉电半写会产生静默脏数据。
 * 块格式: [magic:4][len:2][payload:len][crc32:4], crc 覆盖 magic+len+payload。
 * 读出校验失败 → 返回"损坏", 调用方据此回退默认参数而非执行错误数据。 */
static uint32_t w25_crc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return crc ^ 0xFFFFFFFFu;
}

#define W25_MAGIC   0x57A1D064u   /* 魔数, 标识本工程受保护块 */

void W25Q64_SaveSafe(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (len > 240) len = 240;          /* 单页内(保留头尾) */
    uint8_t blk[256];
    uint16_t p = 0;
    uint32_t magic = W25_MAGIC;
    memcpy(blk + p, &magic, 4); p += 4;
    memcpy(blk + p, &len, 2);   p += 2;
    memcpy(blk + p, data, len); p += len;
    uint32_t crc = w25_crc32(blk, p);
    memcpy(blk + p, &crc, 4);   p += 4;
    W25Q64_EraseSector(addr);
    W25Q64_WritePage(addr, blk, p);
}

uint8_t W25Q64_LoadSafe(uint32_t addr, uint8_t *data, uint16_t maxlen, uint16_t *outlen)
{
    uint8_t blk[256];
    W25Q64_ReadData(addr, blk, 256);
    uint32_t magic; memcpy(&magic, blk, 4);
    if (magic != W25_MAGIC) return 1;              /* 未初始化/魔数不符 */
    uint16_t len;   memcpy(&len, blk + 4, 2);
    if (len > maxlen || len > 240) return 1;       /* 长度异常 */
    uint32_t crc;   memcpy(&crc, blk + 6 + len, 4);
    if (w25_crc32(blk, 6 + len) != crc) return 1;  /* CRC 不符 = 位翻转/半写 */
    memcpy(data, blk + 6, len);
    if (outlen) *outlen = len;
    return 0;
}
