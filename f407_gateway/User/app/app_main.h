#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

/* F407 网关应用层入口
 *   由 Core/Src/main.c 在 USER CODE 保护块内调用：
 *     - APP_MAIN_Init() : MX 外设初始化之后（USER CODE BEGIN 2）
 *   主循环逻辑已迁往 FreeRTOS 任务（freertos.c 的 GW_Task/UI_Task/Alert_Task），
 *   while(1) 由 osKernelStart() 接管，不再有 APP_MAIN_Run。
 */
void APP_MAIN_Init(void);

#endif /* APP_MAIN_H */
