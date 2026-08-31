/* ================================================================
 * F407 网关 UI (OLED) + 菜单 (KEY)
 *   把 g_gw_regs 镜像渲染到 SSD1306, 实时显示系统状态。
 *   使用江协科技移植版 OLED 驱动: OLED_ShowString(Line 1~4, Column 1~16, str)
 *   8x16 字体, 每字占 1 行高(16px) + 1 列宽(8px)。
 *   板载 4 KEY (PA0/PE2/PE3/PE4) 驱动菜单翻页 + 启停/复位。
 *   读 modbus_gateway.h 的 extern g_gw_regs[GW_REG_COUNT]。
 * ================================================================ */

#include <stdio.h>
#include "bsp_oled.h"
#include "bsp_key.h"
#include "modbus_gateway.h"
#include "mb_regmap.h"

/* W25Q64 自检结果 (定义在 app_main.c, FLASH_SelfTest 填充) */
extern uint8_t  g_flash_ok;
extern uint16_t g_flash_id;

static uint32_t s_tick = 0;
static uint8_t  s_page = 0;       /* 0=状态 1=传感 2=计数 */
#define UI_PAGE_COUNT  3

void UI_Init(void)
{
    OLED_Init();
    KEY_Init();      /* 板载 KEY 应用软件初始化 */
}

/* 菜单按键处理：每次主循环都调，保证即时响应 */
static void ui_handle_keys(void)
{
    KEY_ID k = KEY_Scan(0);   /* mode=0 不支持连按 */
    if (k == KEY_NONE) return;

    switch (k)
    {
        case KEY1:   /* PE2 菜单↑ */
            s_page = (s_page + UI_PAGE_COUNT - 1) % UI_PAGE_COUNT;
            break;
        case KEY2:   /* PE3 菜单↓ */
            s_page = (s_page + 1) % UI_PAGE_COUNT;
            break;
        case KEY0:   /* PA0 启动/停：看当前运行状态切换 */
            if (g_gw_regs[R_GW_SYS_STATUS] & 0x0002)   /* bit1=运行 */
                GW_RequestCmd(CMD_STOP);
            else
                GW_RequestCmd(CMD_START);
            break;
        case KEY3:   /* PE4 确认/复位 */
            GW_RequestCmd(CMD_RESET);
            break;
        default:
            break;
    }
}

void UI_Refresh(void)
{
    ui_handle_keys();   /* 每次循环都扫键, 即时响应菜单 */

    /* 节流 ~200ms, 避免每次主循环都刷 I2C */
    if (HAL_GetTick() - s_tick < 200) return;
    s_tick = HAL_GetTick();

    uint16_t sys    = g_gw_regs[R_GW_SYS_STATUS];
    uint16_t status = g_gw_regs[R_GW_MOTOR_STATUS];
    uint16_t fault  = g_gw_regs[R_GW_MOTOR_FAULT];
    uint16_t pwm    = g_gw_regs[R_GW_MOTOR_PWM];
    uint16_t mat    = g_gw_regs[R_GW_MAT_CNT];
    uint16_t sa     = g_gw_regs[R_GW_SORT_A_CNT];
    uint16_t sb     = g_gw_regs[R_GW_SORT_B_CNT];
    uint16_t sens_a = g_gw_regs[R_GW_SENSE_A];
    uint16_t sens_b = g_gw_regs[R_GW_SENSE_B];

    char buf[24];

    /* 翻页时整屏清一次(消除跨页同位置残留); 平时同页只覆盖写, 不整屏清。
       改前每帧都 OLED_Clear() 全屏 1024 字节 @100kHz ≈ 1s 阻塞, 导致主循环卡 1s。
       覆盖写 + 固定宽度后, 每帧只写当前页字符(~560字节) ≈ 0.6s, 主循环不再被长时间卡死。 */
    static uint8_t s_last_page = 0xFF;
    if (s_page != s_last_page)
    {
        OLED_Clear();
        s_last_page = s_page;
    }

    if (s_page == 0)        /* ---- 状态页 ---- */
    {
        OLED_ShowString(1, 1, "F407 GW");
        sprintf(buf, "P%d/3", s_page + 1);
        OLED_ShowString(1, 12, buf);

        /* F103 在线状态: 固定 8 宽左对齐, 避免 ON(7)/OFF(8) 互切时末位 F 残留
           (OFF 比 ON 多 1 字符, 直接 ShowString 覆盖会留前一状态的尾字 'F') */
        sprintf(buf, "%-8s", (sys & BIT_LINK) ? "F103 ON" : "F103 OFF");
        OLED_ShowString(2, 1, buf);

        /* FLASH 自检状态: 统一显实际 JEDEC ID (如 F:EF40)。
           列 9 起 (" F:EF40" 占列 9-15), 避免列 11 起超出 16 字符屏宽把末位 0 截掉。 */
        sprintf(buf, " F:%04X", g_flash_id);
        OLED_ShowString(2, 9, buf);

        /* 状态字段固定 5 宽左对齐, 避免 RUN/STOP/FAULT 互切残留尾字 */
        if      (fault != FAULT_NONE) sprintf(buf, "%-5s", "FAULT");
        else if (status & BIT_RUN)    sprintf(buf, "%-5s", "RUN");
        else                          sprintf(buf, "%-5s", "STOP");
        OLED_ShowString(3, 1, buf);

        /* PC 三态: 未连过(--)/已连且正常(OK)/连上后失联(DOWN) */
        sprintf(buf, "PWM:%3d PC:%-4s", pwm,
                (sys & BIT_PC_SEEN) ? ((sys & BIT_PC_DOWN) ? "DOWN" : "OK") : "--");
        OLED_ShowString(4, 1, buf);
    }
    else if (s_page == 1)   /* ---- 传感页 ---- */
    {
        OLED_ShowString(1, 1, "SENSOR");
        sprintf(buf, "P%d/3", s_page + 1);
        OLED_ShowString(1, 12, buf);

        sprintf(buf, "A:%4d", sens_a); OLED_ShowString(2, 1, buf);
        sprintf(buf, "B:%4d", sens_b); OLED_ShowString(3, 1, buf);
    }
    else                    /* ---- 计数页 ---- */
    {
        OLED_ShowString(1, 1, "COUNT");
        sprintf(buf, "P%d/3", s_page + 1);
        OLED_ShowString(1, 12, buf);

        sprintf(buf, "MAT:%5d", mat);             OLED_ShowString(2, 1, buf);
        sprintf(buf, "A:%4d B:%4d", sa, sb);      OLED_ShowString(3, 1, buf);
    }
}
