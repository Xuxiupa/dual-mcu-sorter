/* ================================================================
 * F407 Modbus 网关 —— 应用层入口
 *   把"系统启动 / 主循环"收口到 APP_MAIN_Init / APP_MAIN_Run，
 *   Core/Src/main.c 只需在 USER CODE 处调用这两个函数。
 * ================================================================ */

#include <stdio.h>
#include <string.h>
#include "main.h"            /* 引入 ITM_SendChar 等 CMSIS 接口 */
#include "modbus_gateway.h"  /* GW_Init / GW_Poll */
#include "ui.h"             /* UI_Init / UI_Refresh (OLED) */
#include "bsp_w25q64.h"     /* W25Q64 SPI Flash (黑匣子存储) */
#include "sys_service.h"    /* WD_Service_Init (系统服务层: 看门狗心跳监督) */

/* W25Q64 自检结果 (UI 状态页显示) */
uint8_t  g_flash_ok = 0;
uint16_t g_flash_id = 0;

/* W25Q64 上电自检: 仅读 JEDEC ID 校验厂商字节, 不做擦除/编程/校验。
 *
 * ★ 关键: 上电阶段绝不能做擦写测试。原因:
 *   main.c 顺序是 MX_IWDG_Init()(第98行, IWDG 2s 超时开始) → APP_MAIN_Init()
 *   → osKernelStart()(第108行, 调度器才起, GW_Task 此后才喂狗)。
 *   APP_MAIN_Init 在调度器之前运行, 这期间无人喂 IWDG。原擦写自检耗时 ~1.8s,
 *   超过 2s 看门狗 → 系统被复位 → OLED 永远稳定不下来(反复复位)。
 *   受保护存储接口(W25Q64_SaveSafe/LoadSafe)保留给运行时(黑匣子)使用,
 *   其可用性由运行时写入验证, 不必在上电阶段破坏性自检。 */
void FLASH_SelfTest(void)
{
    g_flash_id = W25Q64_ReadID();   /* 仅 3 次 SPI 收发, <1ms, 绝对不会拖垮上电 */
    /* 校验厂商字节 == 0xEF (Winbond 全系列一致; 差异在第2字节 memory type)。
       W25Q64_ReadID 返回 mid<<8|did, 故高字节即厂商。 */
    g_flash_ok = ((g_flash_id >> 8) == 0xEF);
}

/* ---------- printf 重定向（双层覆盖）----------
 * Arm Compiler 5 + microLIB 下, printf 实际走 fputc（不是 _write）。
 * 本工程 CubeMX SYS 只配 Serial Wire（PB3 未配 TRACESWO）,
 * Keil Debug Trace Enable 打开后默认 fputc 内部调 ITM_SendChar
 * 进入 while 死锁, 导致 APP_MAIN_Init 永远返回不了。
 * 只改 _write 是没用的, 必须同时覆盖 fputc 为 no-op。
 *
 * 本工程已改走 Modbus Poll 路线, 不依赖调试 printf;
 * 把 fputc + _write 都设成 no-op: 任何 printf 调用安全返回,
 * 不阻塞系统, 也防止未来代码再被 printf 卡死。
 *
 * 若以后想恢复 SWO 打印: CubeMX SYS 改 "Trace Asynchronous Sw"
 * + 打开 Debug (printf) Viewer 后, 用真实 fputc → ITM_SendChar。
 */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    (void)ptr;
    return len;   /* 备用层 */
}

__weak int fputc(int ch, FILE *f)
{
    (void)f;
    return ch;    /* ★ AC5 + microLIB 下 printf 实际走这里, 必须 no-op */
}

/* ================================================================
 * HAL UART 接收完成回调 —— 覆盖 HAL 库默认的 __weak 空实现
 *   USART1 收完 1 字节 → HAL_UART_IRQHandler → 调到这里
 *   必须把字节交给 GW_UART_RxISR, 否则中断来了 s_rb1 还是空的,
 *   主循环永远收不到 PC 的 Modbus 请求, 从站永远不会应答。
 *   之前漏挂这个回调, 链接器将 GW_UART_RxISR 视为 dead code 删除,
 *   导致 Modbus Poll Timeout。★★ 以后新工程不要忘 ★★
 * ================================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    GW_UART_RxISR(huart);   /* 内部按 huart->Instance 区分 USART1 / USART3 */
}

/* ================================================================
 * 应用初始化：在 main.c 的 MX 外设初始化（MX_USARTx_Init 等）之后调用
 *   只做外设级初始化（缓冲/OLED/KEY）。RXNE 中断接收由 FreeRTOS 的
 *   gw_task 入口调 GW_StartRx() 启动（队列创建之后，ISR 入队才安全）。
 * ================================================================ */
void APP_MAIN_Init(void)
{
    GW_Init();   /* 环形缓冲 + 寄存器镜像清零（不启动 RXNE，见 GW_StartRx） */
    UI_Init();   /* 初始化 OLED 并显示 (I2C1 PB8/PB9, P3 排母) */

    /* W25Q64 SPI Flash 自检 (读 ID + 擦写校验), 结果在 UI 状态页显示 */
    W25Q64_Init();
    FLASH_SelfTest();

    /* 系统服务层: 看门狗心跳数组清零 (调度器启动前, wd_task 巡检用) */
    WD_Service_Init();

    /* 不再 printf: 调试信息统一通过 Modbus Poll 读寄存器镜像 */
}
