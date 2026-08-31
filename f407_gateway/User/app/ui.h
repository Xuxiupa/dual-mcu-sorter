#ifndef UI_H
#define UI_H

void UI_Init(void);     /* 初始化 OLED + 清屏 */
void UI_Refresh(void);  /* 读 g_gw_regs 渲染到 OLED (内部 200ms 节流) */

#endif /* UI_H */
