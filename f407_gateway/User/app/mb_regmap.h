#ifndef MB_REGMAP_H
#define MB_REGMAP_H

/* ============================================================================
 * 双 MCU 物料分拣系统 - 共享 Modbus 寄存器映射
 *
 * 本文件被 BOTH 端包含：
 *   - F103 端：作为 Modbus 从站（地址 0x01），实现 read/write 回调
 *   - F407 端：作为网关（对 F103 作主站轮询；对 PC 作从站 0x02，应答 PC）
 *
 * 所有寄存器均为 16-bit 保持寄存器（Holding Register）。
 * 修改本文件后，两端必须同步重新编译，否则协议对不上。
 * ==========================================================================*/

/* ---- F103 从站，Modbus 地址 0x01 ---- */
#define F103_ADDR            0x01
#define R_F103_TARGET        0   /* W : 目标 PWM / 速度指令   0..999            */
#define R_F103_ACT_PWM       1   /* R : 实际输出 PWM          0..999            */
#define R_F103_STATUS        2   /* R : 状态位  bit0=运行 bit1=故障 bit2=分拣中 */
#define R_F103_FAULT         3   /* R : 故障码  0无 1失速 2超温 3堵料           */
#define R_F103_MAT_CNT       4   /* R : 物料累计计数                            */
#define R_F103_SORT_CMD      5   /* W : 分拣指令 0无 1→A料道 2→B料道           */
#define R_F103_SENSE_A       6   /* R : 反射红外/光敏采样（原始 ADC 值）        */
#define R_F103_SENSE_B       7   /* R : 热敏电阻采样（原始 ADC 值）             */
#define R_F103_CTRL          8   /* W : 控制位 bit0=启动 bit1=停止 bit2=复位    */
#define R_F103_POST          9   /* R : POST 上电自检结果位掩码                */
#define F103_REG_COUNT       10  /* 寄存器总数                                 */

/* ---- F407 网关从站（对 PC），Modbus 地址 0x02 ---- */
#define GW_ADDR              0x02
#define R_GW_MOTOR_PWM       0   /* R : 镜像 F103 实际 PWM                      */
#define R_GW_MOTOR_TARGET    1   /* R/W : PC 下发的目标速度指令 0..999          */
#define R_GW_MOTOR_STATUS    2   /* R : 电机状态（镜像 F103）                   */
#define R_GW_MOTOR_FAULT     3   /* R : 故障码（镜像 F103）                     */
#define R_GW_MAT_CNT         4   /* R : 物料累计计数（镜像）                    */
#define R_GW_SORT_A_CNT      5   /* R : A 料道分拣计数                          */
#define R_GW_SORT_B_CNT      6   /* R : B 料道分拣计数                          */
#define R_GW_SYS_STATUS      7   /* R : 系统状态 bit0=F103在线 bit1=运行 bit4=PC掉线 */
#define R_GW_SENSE_A         8   /* R : 最新传感器 A（镜像）                    */
#define R_GW_SENSE_B         9   /* R : 最新传感器 B（镜像）                    */
#define R_GW_CMD             10  /* W : 系统指令 bit0=启动 bit1=停止 bit2=复位  */
#define R_GW_POST            11  /* R : POST 上电自检结果(镜像 F103)            */
#define R_GW_LOG_CNT         12  /* R : 黑匣子累计事件数(掉电不丢)              */
#define R_GW_LOG_LAST        13  /* R : 最近一条事件码(BB_EV_*)                 */
#define R_GW_LOG_TS          14  /* R : 最近事件时间戳(秒, 低16位)             */
#define R_GW_SORT_CMD        15  /* W : PC 远程触发分拣 0无 1→A料道 2→B料道      */
#define GW_REG_COUNT         16  /* 寄存器总数                                 */

/* 状态/控制位定义（便于两端统一语义） */
#define BIT_RUN              0x0001   /* 运行        */
#define BIT_FAULT            0x0002   /* 故障        */
#define BIT_SORTING          0x0004   /* 分拣进行中  */
#define BIT_GW_DOWN          0x0008   /* F103 感知网关掉线(本地安全态触发) */
#define BIT_PC_DOWN          0x0010   /* PC 心跳超时/上位机失联(R_GW_SYS_STATUS bit4) */
#define BIT_PC_SEEN          0x0020   /* PC 至少连过一次(R_GW_SYS_STATUS bit5): 区分"未连接"与"掉线" */
#define BIT_LINK             0x0001   /* F103 在线   */

/* 控制/系统指令位（R_GW_CMD / R_F103_CTRL 共用语义）
 *   bit0=启动  bit1=停止  bit2=复位 */
#define CMD_START            0x0001
#define CMD_STOP             0x0002
#define CMD_RESET            0x0004

#define SORT_NONE           0
#define SORT_BIN_A          1
#define SORT_BIN_B          2

#define FAULT_NONE          0
#define FAULT_STALL         0x0001   /* 失速 */
#define FAULT_OVERHEAT      0x0002   /* 超温 */
#define FAULT_BLOCKED       0x0004   /* 堵料 */
#define FAULT_SEN_TEMP      0x0008   /* 热敏(PA4)断线/短路 */
#define FAULT_SEN_LIGHT     0x0010   /* 光敏(PB0)断线/短路 */
#define FAULT_SEN_REFLECT   0x0020   /* 反射红外(PB1)断线/短路 */
#define FAULT_POST          0x0040   /* POST 上电自检未全部通过 */

/* POST 上电自检结果位 (R_F103_POST / R_GW_POST) */
#define POST_ADC            0x0001   /* ADC 3路扫描正常(时钟+参考正常) */
#define POST_MOTOR          0x0002   /* 电机/TB6612 PWM 外设已配置      */
#define POST_ENC            0x0004   /* 编码器/旋钮 TIM2 已配置         */
#define POST_SERVO          0x0008   /* 舵机(当前未实装, 预留)          */
#define POST_ALL            0x0007   /* 当前可实现三项全过               */

#endif /* MB_REGMAP_H */
