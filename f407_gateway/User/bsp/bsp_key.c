/* ================================================================
 * 板载 KEY 驱动 (PA0 / PE2 / PE3 / PE4, 低有效)
 *   GPIO 配置 (GPIO_Input, Pull-up) 由 CubeMX 生成的 MX_GPIO_Init()
 *   统一完成, 本文件只做消抖状态机 + 读引脚, 不手写 HAL_GPIO_Init。
 *   消抖方案: 经典状态机 (key_up 标志 + HAL_Delay(10) 去抖)。
 *   注: HAL_Delay 在裸机/RTOS 前均可正常工作, 依赖 SysTick。
 * ================================================================ */

#include "bsp_key.h"
#include "stm32f4xx_hal.h"

static uint8_t s_key_up = 1;   /* 1=可检测新按键, 0=本次按键已消费 */

void KEY_Init(void)
{
    /* KEY0/1/2/3 (PA0/PE2/PE3/PE4) 的 GPIO 已在 CubeMX MX_GPIO_Init()
     * 里配为 GPIO_Input + Pull-up, 此处不重复初始化, 仅复位软件消抖标志。 */
    s_key_up = 1;
}

KEY_ID KEY_Scan(uint8_t mode)
{
    if (mode) s_key_up = 1;   /* mode=1 连按: 强制允许重复触发 */

    if (s_key_up &&
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET ||
         HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET ||
         HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET ||
         HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET))
    {
        HAL_Delay(10);   /* 10ms 去抖 */
        s_key_up = 0;

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) return KEY0;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET) return KEY1;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET) return KEY2;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET) return KEY3;
    }
    else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET &&
             HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET &&
             HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET &&
             HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET)
    {
        s_key_up = 1;   /* 全部松开 → 恢复可触发 */
    }

    return KEY_NONE;
}
