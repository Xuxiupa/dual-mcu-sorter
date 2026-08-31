#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdint.h>

/* 板载 4 个 KEY (原理图确认, 低有效):
 *   KEY0 = PA0  (WKUP)  → 启动/停止
 *   KEY1 = PE2          → 菜单 ↑
 *   KEY2 = PE3          → 菜单 ↓
 *   KEY3 = PE4          → 确认/复位
 */
typedef enum {
    KEY_NONE = 0,
    KEY0 = 1,   /* PA0  启动/停 */
    KEY1 = 2,   /* PE2  菜单↑   */
    KEY2 = 3,   /* PE3  菜单↓   */
    KEY3 = 4    /* PE4  确认/复位 */
} KEY_ID;

void KEY_Init(void);

/* 软件消抖扫描。mode=0 不支持连按(同一键按住只返回一次),
 * mode=1 支持连按(按住期间每次调用都返回)。返回按下的 KEY_ID */
KEY_ID KEY_Scan(uint8_t mode);

#endif /* BSP_KEY_H */
