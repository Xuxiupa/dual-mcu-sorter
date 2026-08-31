# 双 MCU 工业物料分拣传送带系统

> STM32F407（协议网关 / FreeRTOS） + STM32F103C8T6（执行 / 传感端），Modbus RTU 级联，PC 上位机实时监控与下发控制。

本项目是一个**可上板运行的完整嵌入式系统**，覆盖：电机开环调速、SG90 舵机自动分拣、多路传感器融合、Modbus RTU 主从级联、FreeRTOS 多任务、看门狗系统服务层、掉电不丢故障黑匣子、断链指令补发、以及 Python 上位机。代码全部经上板闭环验证。

系统架构图见 [`架构图.svg`](./架构图.svg)。

---

## 1. 系统架构总览

```
┌──────────────┐   Modbus RTU     ┌──────────────────────────┐   Modbus RTU     ┌──────────────────────┐
│   PC 上位机   │ ════════════════ │   F407 网关 (从站 0x02)    │ ════════════════ │  F103 执行端 (从站 0x01) │
│ GUI/ModbusPoll│   USB↔CH340      │  FreeRTOS 五任务 + 黑匣子  │   交叉串口+GND    │  电机/舵机/传感器/分拣   │
└──────────────┘   (COM14)        └──────────────────────────┘   (COM11调试)     └──────────────────────┘
```

- **F407** 是协议网关：对 PC 是 Modbus **从站**（地址 0x02，应答上位机）；对 F103 是 Modbus **主站**（轮询 0x01）。
- **F103** 是执行 / 传感端：电机调速、舵机分拣、传感器采样均在本地完成，不依赖网关即可独立安全停机（fail-safe）。
- 三层通信全部为 **Modbus RTU（半双工 9600 8N1）**，无 LWIP / 以太网。

---

## 2. 硬件清单（BOM）

| 角色 | 型号 | 说明 |
|------|------|------|
| 主控网关 | STM32F407ZET6 开发板 | 板载 CH340G（USART1→USB），I2C 接 OLED，TIM13 出 BEEP |
| 执行 / 传感 | STM32F103C8T6 面包板 | USART2=Modbus，TIM1=电机PWM，TIM3=舵机，ADC 三路，PA12=对射EXTI |
| 电机驱动 | TB6612 | AIN1/AIN2=PB3/PB4，STBY 须接高电平 |
| 直流减速电机 | 普通有刷直流电机 | PA8 TIM1 PWM 开环调速 |
| 舵机 | SG90 | PA6 TIM3，0°/90°/150° 对应回中/A料道/B料道 |
| 对射光电 | 分体式（发射+接收） | OUT→PA12，下降沿触发物料计数 |
| 光敏电阻 | LDR | PB0 ADC，物料分类依据 |
| 反射红外 | 红外对管 | PB1 ADC，分拣复检 |
| 热敏电阻 | NTC | PA4 ADC，超温停机 |
| Flash | W25Q64 | SPI，故障黑匣子（掉电不丢） |
| 显示 | 0.96" SSD1306 OLED | I2C（PB8/PB9，地址 0x3C） |

> 串口接线：F407 `USART1 PA9/PA10` → 板载 CH340G → USB（本机识别为 **COM14**）；
> F103 `USART1 PA9/10` → USB-TTL 调试 printf（本机 **COM11**）；
> F407↔F103：`USART3 PB10/11` ↔ `USART2 PA2/3` 交叉接，共 GND。COM 号以设备管理器为准。

---

## 3. 通信协议 — Modbus RTU 寄存器映射

所有寄存器为 **16-bit 保持寄存器（Holding Register）**，小端字节序。两端共享 `mb_regmap.h`（已分叉：F407 版含全部网关寄存器，F103 版只含执行端寄存器）。

### 3.1 F103 执行端（从站地址 0x01，共 10 寄存器）

| 地址 | 宏 | 方向 | 含义 |
|------|-----|------|------|
| 0 | `R_F103_TARGET` | W | 目标 PWM / 速度指令 0..999 |
| 1 | `R_F103_ACT_PWM` | R | 实际输出 PWM 0..999 |
| 2 | `R_F103_STATUS` | R | 状态位（bit0=运行 bit1=故障 bit2=分拣中 bit3=网关掉线 bit4=复检OK bit5=复检Fail） |
| 3 | `R_F103_FAULT` | R | 故障码（详见 §5.2） |
| 4 | `R_F103_MAT_CNT` | R | 物料累计计数 |
| 5 | `R_F103_SORT_CMD` | W | 分拣指令 0无 / 1→A料道 / 2→B料道 |
| 6 | `R_F103_SENSE_A` | R | 反射红外 / 光敏原始 ADC 值 |
| 7 | `R_F103_SENSE_B` | R | 热敏电阻原始 ADC 值 |
| 8 | `R_F103_CTRL` | W | 控制位 bit0=启动 bit1=停止 bit2=复位 |
| 9 | `R_F103_POST` | R | 上电自检（POST）结果位掩码 |

### 3.2 F407 网关（对 PC 从站地址 0x02，共 16 寄存器）

| 地址 | 宏 | 方向 | 含义 |
|------|-----|------|------|
| 0 | `R_GW_MOTOR_PWM` | R | 镜像 F103 实际 PWM |
| 1 | `R_GW_MOTOR_TARGET` | R/W | PC 下发目标速度 0..999 |
| 2 | `R_GW_MOTOR_STATUS` | R | 电机状态（镜像 F103） |
| 3 | `R_GW_MOTOR_FAULT` | R | 故障码（镜像 F103） |
| 4 | `R_GW_MAT_CNT` | R | 物料累计计数（镜像） |
| 5 | `R_GW_SORT_A_CNT` | R | A 料道分拣计数 |
| 6 | `R_GW_SORT_B_CNT` | R | B 料道分拣计数 |
| 7 | `R_GW_SYS_STATUS` | R | 系统状态 bit0=在线 bit1=运行 bit4=PC掉线 bit5=PC曾连接 |
| 8 | `R_GW_SENSE_A` | R | 最新传感器 A（镜像） |
| 9 | `R_GW_SENSE_B` | R | 最新传感器 B（镜像） |
| 10 | `R_GW_CMD` | W | 系统指令 bit0=启动 bit1=停止 bit2=复位 |
| 11 | `R_GW_POST` | R | POST 上电自检结果（镜像 F103） |
| 12 | `R_GW_LOG_CNT` | R | 黑匣子累计事件数（掉电不丢） |
| 13 | `R_GW_LOG_LAST` | R | 最近一条事件码（`BB_EV_*`） |
| 14 | `R_GW_LOG_TS` | R | 最近事件时间戳（秒，低 16 位） |
| 15 | `R_GW_SORT_CMD` | W | PC 远程触发分拣 0无 / 1→A料道 / 2→B料道 |

**故障码 `R_*_FAULT`**：`0x01` 失速 / `0x02` 超温 / `0x04` 堵料 / `0x08` 热敏断线 / `0x10` 光敏断线 / `0x20` 反射断线 / `0x40` POST 未通过。

---

## 4. 固件结构

### 4.1 F407 网关（FreeRTOS CMSIS_V2，5 个应用任务 + 1 占位）

| 任务 | 优先级 | 职责 |
|------|--------|------|
| `GW_Task` | 高 | 网关核心：RX 字节队列 → 主站轮询 F103 + 从站应答 PC + 数据镜像 + LED 指示；20ms 周期 |
| `UI_Task` | 低 | OLED 渲染 + KEY 扫描（内部 200ms 节流，慢 I2C 不抢 gw 时间） |
| `Alert_Task` | 中 | BEEP 蜂鸣 + LED0 报警；监听 PC 掉线 / F103 故障事件 |
| `BB_Task` | — | 故障黑匣子异步持久化到 W25Q64（事件码 + 时间戳，掉电不丢） |
| `WD_Task` | 最低(Idle) | **系统服务层看门狗**：每秒巡检 4 业务任务心跳，独占 IWDG 喂狗 |
| `defaultTask` | Normal | CubeMX 占位空转任务 |

队列：`g_q_rx`（ISR `xQueueSendFromISR` 入字节）→ gw_task；`g_q_ev`（事件）→ alert_task；`g_q_bb`（黑匣子事件）→ bb_task。

### 4.2 F103 执行端（裸机主循环）

- **电机**：PA8 TIM1 PWM 开环调速（0~999 → 占空比），TB6612 方向 PB3/PB4。
- **舵机**：PA6 TIM3，定时 20ms 周期，1.5ms 中值=回中，分拣 A/B 拨到固定角。
- **传感器**：ADC 三路独立采样 — 热敏 PA4 / 光敏 PB0 / 反射 PB1（逐通道重配 Rank1，规避 CubeMX 不连续模式坑）。
- **分拣逻辑**：PA12 对射下降沿（双层去抖：上电 1s 宽限 + 500ms 重触发抑制）→ 光敏 PB0 超阈值分 A/B → 舵机拨料 → 窗口内 PB1 反射复检（置状态位，不丢真实物料）。
- **安全**：网关静默 >2s（GW_SILENCE_MS）→ 独立安全停机（不依赖 F407 通知）。

---

## 5. PC 上位机工具（Python）

位于 `stm32f407_modbus_gateway/tools/`：

| 文件 | 作用 |
|------|------|
| `sorter_gui.py` | 主界面（tkinter + matplotlib）：实时寄存器面板、状态灯（中文）、SENSE 历史曲线、PWM 滑条、分拣/启停/复位下发、配置持久化（`~/.sorter_gui.json`）、黑匣子 CSV/JSON 导出 |
| `live_view.py` | 轻量命令行实时寄存器查看 |
| `regmap_defs.py` | PC 端数据层真源（寄存器名 / 事件名 / 位定义），被 GUI 引用 |
| `run_gui.bat` / `run_gui_silent.vbs` | Windows 一键启动 |

> 依赖：本机若用 MSYS2 的 Python，优先 `pacman -S mingw-w64-x86_64-python-pyserial`（加 matplotlib 同理）；GUI 曲线需 `matplotlib`。

---

## 6. 构建与烧录

### 6.1 F407 网关（Keil + ST-Link）

1. 打开 `stm32f407_modbus_gateway/MDK-ARM` 工程。
2. 确认 `User/app/sys_service.c` 已加入 build target（新增文件需手动加）。
3. **`Project → Rebuild all`**（勿用 F7 增量，可能跳过改动文件）。
4. `F8` 下载，确认输出 **`Verify OK`**。
5. 若经 CubeMX 重新 Generate：检查 `main.c` 的 `USER CODE BEGIN 2/3` 是否仍调用 `APP_MAIN_Init()/Run()`，并 grep `stm32f4xx_it.c` 防 USART IRQ 重复定义（`#247 already defined`）。

### 6.2 F103 执行端（Keil + ST-Link）

1. 打开 `stm32f103_motor_servo_adc/MDK-ARM` 工程。
2. **`Rebuild all` → 0 Error(s)** → `F8` **Verify OK** → 烧录。
3. 串口调试（printf）走 `USART1` → USB-TTL（COM11）。

> ⚠️ Keil 编辑器缓存陷阱：AI 用工具直接改了磁盘 `.c`，但 Keil 标签页可能仍显示旧版（带 `*`）。务必关标签页重开 → 肉眼确认改动 → `Ctrl+S` → Rebuild all → 确认 `Verify OK`，否则烧的是旧码。

---

## 7. 工程收口改进项（最近一轮）

| # | 改进 | 关键点 |
|---|------|--------|
| 1 | 看门狗事件 7 落盘漏洞 | IWDG 复位会打断 W25Q64 写；改用 **BKPSRAM 备份域旁路**（原子写 <1us，复位不丢），重启后由 `GW_Poll` 迁移进黑匣子 |
| 2 | GUI 状态显示 | 排查确认 SYS_STATUS 文本与状态灯本就一致；仅把物理串口连接灯标签改为"串口连接"以区分"PC掉线"语义 |
| 3 | PC 远程触发分拣 | 网关新增透传寄存器 `R_GW_SORT_CMD(15)`，写操作经下行补发 FIFO 转发 F103；GUI 加 3 个分拣按钮 |
| 4 | 串口直驱舵机测试 | F103 `app_main.c` 新增 `V<deg>` 定角度命令，隔离"供电 vs 信号"问题；`A/B` 触发分拣、`CL` 打印原始值 |
| 5 | 分拣 / 复检阈值标定 | 阈值由 `#define` 改为运行期变量 + 访问器；`TL <n>` / `TR <n>` 串口实时标定，无需重编译 |

---

## 8. 已知设计决策 / 限制

- **开环 PWM 调速**：PA0/1 接的是 EC11 手动旋钮（非测速编码器），无速度闭环，保留开环。
- **阈值占位值**：光敏分类 / 反射复检阈值需按真实光环境用 `TL/TR` 现场标定。
- **SYS_STATUS 位语义**以 `mb_regmap.h` 为准；F103 与 F407 两端的 `mb_regmap.h` 已按职责分叉（F407 含全部网关寄存器）。
- **PC↔F407 串口**：Windows + pyserial + CH340 下 `ser.write()` 偶发阻塞 3~12s，靠固件端 **1s 稳定确认** 瞬态抑制根治，而非拉长 PC 超时。

---

## 9. 快速验证清单

- [ ] F407：加 `sys_service.c` → Rebuild → Verify OK → 烧录。
- [ ] F103：Rebuild 0 Error → Verify OK → 烧录。
- [ ] 上位机连 COM14，面板实时刷新；点启停/分拣，舵机动作、计数增长。
- [ ] 拔 USB → 约 13s 蜂鸣 + 上位机自动重连 + 黑匣子记"PC失联→恢复"。
- [ ] 故意 `GW_Task` 加 `osDelay(15000)` → 板子周期性重启（看门狗闭环），**验证后必须删除该行重烧**。
