# C 板固件说明（与代码一致）

基于 STM32F407 的 RoboMaster C 型开发板固件，作为 RS485 Modbus RTU 从站，对外同时提供 **风机控制**（PWM + FG 反馈）与 **底盘电机控制**（CAN + 运动学解算）两套互不干扰的子系统。

> 本文档以代码（`bsp/boards/bsp_modbus.c` 中的 `REG_*` 宏与状态机、`Src/tim.c` 中的 TIM 配置、`Src/stm32f4xx_it.c` 中的中断、`cmake/stm32cubemx/CMakeLists.txt` 中的构建清单）为唯一准绳。`接口文档.md` 是寄存器层面的详细定义。

## 1. 项目概述

本固件面向 RoboMaster C 型开发板，**同时**承担两类上层控制请求：

- **风机子系统**：4 路 FG 反馈 + 3 路同步 PWM 输出，与 EBS-P300 / ROS2 上位机通过 Modbus RTU 通信。
- **电机子系统**：4 个达妙电机 CAN 1 Mbps 控制，运行项目的正交全向轮运动学解算。

硬件平台：**STM32F407IGH6**（UFBGA176 封装，1 MB Flash / 128 KB SRAM + 64 KB CCMRAM）——
以 `.ioc` 的 `Mcu.CPN` 与 `STM32F407xx_FLASH.ld` 的存储器布局为准。
软件架构：FreeRTOS **5 个任务**（`ModbusTask` / `FanTask` / `InsTask` / `PulseTask` / `LedTask`），1 ms HAL 时基（TIM6）+ FreeRTOS SysTick 节拍（`xPortSysTickHandler`），Cortex-M4F 168 MHz。

### 参考资料

| 资料 | 出处 |
|---|---|
| 《RoboMaster 开发板 C 型用户手册》 | 官方下载页：<https://www.robomaster.com/zh-CN/products/components/general/development-board-type-c#downloads> |

> 该 PDF 约 1.9 MB，**不入库**（`.gitignore` 已排除 `RoboMaster*.pdf`），需要时按上面的链接自行下载。
> 引脚定义、CAN 口线序、加热电路规格等均以该手册与代码为准。

## 2. 硬件连接

### 2.1 PWM/FG 资源映射

| 接口 | 定时器通道 | STM32 引脚 | 功能 | 对应风机 |
|------|------------|-----------|------|----------|
| PWM1 | TIM1_CH1 | PE9  | FG 输入捕获 | 风机 1 反馈 |
| PWM2 | TIM1_CH2 | PE11 | FG 输入捕获 | 风机 2 反馈 |
| PWM3 | TIM1_CH3 | PE13 | FG 输入捕获 | 风机 3 反馈 |
| PWM4 | TIM1_CH4 | PE14 | FG 输入捕获 | 风机 4 反馈 |
| PWM5 | TIM8_CH1 | PC6  | PWM 输出     | 风机 1 控制 |
| PWM6 | TIM8_CH2 | PI6  | PWM 输出     | 风机 2 控制 |
| PWM7 | TIM8_CH3 | PI7  | PWM 输出     | 风机 3 + 风机 4 共用控制 |

约束：

- 1/5 = 风机 1，2/6 = 风机 2，3/7 = 风机 3，4/7 = 风机 4。
- 风机 3 和 4 共用 PWM7（取两者较大占空比输出），逻辑编号仍是 1/2/3/4。
- FG 输入使用 STM32 内部上拉（`GPIO_PULLUP`），适合 FG/开集电极型信号；输入必须兼容 3.3V，5V 信号需电平转换。

### 2.2 RS485 接线（USART6 / 外壳丝印 UART1 的 3-Pin 口）

| 信号     | 引脚 | 说明                            |
|----------|------|---------------------------------|
| RS485_TX | PG14 | USART6 TX（AF8）→ 模块 RXD/DI   |
| RS485_RX | PG9  | USART6 RX（AF8）← 模块 TXD/RO   |
| RS485_RE/DE | PG8 | 高=发送，低=接收（方向控制，需从板内另引） |

接口：C 板外壳丝印的 **UART1（3-Pin）**，脚序 `GND - TXD - RXD`。

> ⚠️ **C 板丝印与 MCU 外设交叉错位**（官方手册原文）：
> "开发板的外壳丝印（UART1 与 UART2）与 STM32 的实际串口配置并不对应，
> **外壳丝印 UART1 对应 STM32 的 UART6，外壳丝印 UART2 对应 STM32 的 UART1**。"
>
> | 外壳丝印 | 实际 MCU 外设 | 引脚 | 脚序 | 供电 |
> |---|---|---|---|---|
> | UART1 (3-Pin) | **USART6** | PG14 / PG9 | GND-TXD-RXD | 无 |
> | UART2 (4-Pin) | USART1 | PA9 / PB7 | RXD-TXD-GND-5V | 5V |
>
> **3-Pin 口没有电源脚**，TTL-RS485 模块需另找 5V/3.3V 供电。

默认串口参数：**115200, 8N1**，从站地址 **1**（可由寄存器 `0x0091` 改为 1~247，写 0 或 >247 回退 1）。
波特率/校验可由寄存器 `0x0090` 枚举改（8 种组合，见接口文档 §7.2），上电读一次，复位生效。

> 历史：早期版本走 USART2（PA2/PA3）；后短暂改为 USART1（4-Pin 口）；最终确定用 3-Pin 口，
> MCU 侧为 USART6。

> **RS485 收发实现（2026-09-10 起对齐 running）**：**不使用 HAL UART 中断状态机**，
> 而是在 `USART6_IRQHandler` 里裸寄存器自管：RXNE 读 DR 入环形 FIFO、**每次中断都清 ORE/FE/NE/PE**、
> TXE 逐字节发送、**TC 中断才切回接收方向**（PG8）。这样从架构上消除了
> "HAL 逐字节接收遇 ORE 溢出后永久停摆"与"发送未移完就拉低方向脚截断末字节"两个隐患。

### 2.3 CAN1 总线

| 信号  | 引脚 | 说明                            |
|-------|------|---------------------------------|
| CAN_TX| PD1  | CAN1 TX，AF9                    |
| CAN_RX| PD0  | CAN1 RX，AF9                    |

波特率：**1 Mbps**，位时间 14 TQ（BRP=2，TS1=11TQ，TS2=2TQ，SJW=1TQ，采样点 85.7%）。
**自动重传开启**（`AutoRetransmission = ENABLE`，对齐 running），偶发错误帧能自动补发；
总线无应答时会占满三个邮箱，由发送函数"邮箱满 → 逐个 `HAL_CAN_AbortTxRequest`"兜底。

收发走 **HAL API + 中断队列**（对齐 running）：`HAL_CAN_AddTxMessage` 发送，
`CAN1_RX0` 中断里 `HAL_CAN_RxFifo0MsgPendingCallback` **只把帧写入 FreeRTOS 队列**，
寄存器更新在任务上下文完成。

> **重要**：早期版本曾把 CAN1 配到 PA11/PA12，那两个脚实际是 USB_OTG_FS_DM/DP。**C 板的 CAN1 在 PD0/PD1**，以 RoboMaster C 板用户手册为准。

### 2.4 不再使用的接口

- **USB CDC 虚拟串口**：已整体下线。主因是 PA11/PA12 实际是 USB 引脚，PD0/PD1 才是 CAN。
  - **已从仓库删除**：`Src/usb_device.c`、`Src/usbd_conf.c`、`Src/usbd_desc.c`、`Src/usbd_cdc_if.c`，
    以及 `Middlewares/ST/STM32_USB_Device_Library/`。
  - **仍在盘上、但未参与构建**：`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c`、
    `stm32f4xx_hal_pcd_ex.c`、`stm32f4xx_ll_usb.c`。这三个是 HAL 库原件，CubeMX 重新生成会拉回，故保留。
  - 是否参与构建，以 `cmake/stm32cubemx/CMakeLists.txt` 的源文件清单为准（第 62~63 行有说明）。
  - 固件体积从 56 KB 降到 31.7 KB。

## 3. 软件架构（FreeRTOS 5 任务）

| 任务 | 周期 | 优先级 | 职责 |
|------|------|--------|------|
| ModbusTask | 5 ms | +4 | RS485 帧解析（裸寄存器中断收发）、Modbus 03/06/10/04 协议、**风机占空比输出**（自动前馈 / 手动跟随）、CAN 电机发送（HAL API）、CAN 接收队列排空、心跳超时、参数落盘 |
| FanTask    | 5 ms | +3 | 调 `fan_tick()` —— ⚠️ **当前为空转**：它驱动的斜坡状态机（`bsp_fric.c`）恒为 `IDLE`，因为 `fan_set_target()` / `fan_stop()` **全工程无人调用**。风机输出实际由 ModbusTask 完成 |
| InsTask    | ~1 ms（DRDY 驱动）| +3 | BMI088 采样 → Mahony 姿态解算 → 发布 seqlock 快照；恒温 PID（TIM10）；栈 512 字（含 sqrt/atan2 浮点库调用） |
| PulseTask  | 10 ms| +2 | 500 ms 窗口 FG 测频、RPM 计算 |
| LedTask    | 5 ms | +1 | RGB 倾斜指示灯（色相=倾斜方向，饱和度=幅度，亮度=系统状态） |

> ⚠️ **风机输出的真实路径**：`ModbusTask` → `modbus_poll()` → `fan_auto_update()`（自动模式，
> 每 5 ms 算一次并直接写 TIM8 CCR）/ `sync_fan_outputs_from_regs()`（手动模式，跟随 `0x0100~0x0103`）。
> **不要以为是 FanTask 在输出** —— 它那套斜坡逻辑目前没有被接上。

时基分工（与 `running` 对齐）：

- **SysTick**（1 kHz）→ FreeRTOS 节拍，由 `SysTick_Handler` 自实现（`Src/stm32f4xx_it.c`），不在编译 `cmsis_os2.c` 的前提下提供 `xPortSysTickHandler()`。
- **TIM6**（1 kHz）→ HAL 时基，由 `Src/stm32f4xx_hal_timebase_tim.c` 提供 `HAL_InitTick()`，优先级 15（最低）。
- 已用定时器：TIM1（FG 捕获）、TIM5（bsp_led 自用）、TIM6（HAL 时基）、TIM8（风机 PWM）。

## 4. 通信协议概要

详细寄存器定义见 `接口文档.md`。下表仅作提纲：

| 协议地址 | 用途 | 读写 |
|----------|------|------|
| 0x0000~0x0003 | 电机 1~4 使能 | R/W |
| 0x0004~0x0007 | 电机 1~4 清错 | R/W（边沿触发）|
| 0x000A~0x0011 | 四轮目标角速度 float32 | R（由运动学覆盖）|
| 0x0014~0x0019 | vy / vx / wz float32 | R/W（上位机下发）|
| 0x0040~0x008F | 电机状态区（只读）⚠️ 真正有数据的是 **0x0064~0x008B** | R |
| 0x0090~0x00BF | 参数区（UART_CFG / SLAVE_ADDR / 用户参数）| R/W |
| 0x0100~0x0103 | 风机 1~4 占空比 0~100（手动）| R/W |
| 0x0110~0x0137 | 风机 1~4 状态区（每块 10 寄存器，固件每 5 ms 刷新）| R |
| 0x0140~0x014F | 风机自动模式参数（姿态前馈）| R/W |
| 0x0150~0x015F | IMU 姿态输出（欧拉角 / 陀螺 / **`0x015C` 自动前馈占空比** / 状态 / 温度）| R |
| 0x0160~0x016C | 风机查表 `LUT[0..12]`（θ = 0°,15°,…,180°）| R/W |
| 0x016D | LUT 魔数（`0xA5C3` = 表有效）| R/W |
| 0x016E~0x016F | 实时总倾角 θ（float32）| R |
| 0x0170~0x0173 | 保持区尾部保留（恒 0）| R |
| 0x0174~0x017B | 姿态四元数（4×float32，w/x/y/z）| R |

> **断电保持区 = `0x0090~0x0173` 整段**（2026-09-15 由 `0x0090~0x00BF` 扩展；
> 2026-09-16 修复"CRC 把自身算进校验范围"导致**断电保持从未生效**的 bug，`VERSION` 升为 **3**，见 `接口文档.md` §7.3）。
> 四元数（`0x0174~0x017B`）**刻意放在保持区之外** —— 它是每 5 ms 刷新的实时量，持久化没有意义。

**Modbus 功能码支持**：0x03（读保持寄存器）、0x04（读输入寄存器，复用 0x03 语义）、0x06（写单寄存器）、0x10（写多寄存器）。

**Modbus 异常码**：0x02 非法地址（写状态区或越界）、0x04 设备故障（CRC 错误等）。

**写权限隔离**：Modbus 写只读区（`0x0040~0x008F` 状态区、`0x0150~0x015F` / `0x016E~0x016F` / `0x0174~0x017B`）
会被静默忽略；保持区（`0x0090~0x0173`）任意写入触发 2 s 防抖后落盘 Flash A/B 双区（0x080C0000 / 0x080E0000）。

## 5. 运动学解算

底盘为**正交全向轮**：LF/RR 是 X 轮（只吃 vx），RF/LR 是 Y 轮（只吃 vy），中心布局 0.15 m × 0.15 m，轮半径 0.05 m。**最大轮速 ±50 rad/s，等比例限幅**（纯安全钳位；上位机 Fast 档最坏组合 vx=2.0 / vy=1.2 / wz=4.0 只到 46.0 rad/s，全程不触发；旧值 30 会把 Fast 档 vx 削掉 25%）。

权威公式参考：`D:\stm32\running\Core\Src\kinematics.c`。**注意**：`running\RM_C_Board_Template\...\chassis_kinematics.c` 的公式不同（旋转项系数 1.0 而非 1.5），不能当参考。

通信丢失保护：上位机每次写 0x0014~0x0019 刷新心跳；500 ms 内无新命令强制 4 轮速度为 0，NaN/Inf 视为 0。

## 6. CAN 电机协议

标准 CAN 数据帧，**1 Mbps**，与 `running` 一致：

| 帧 | ID | 字节内容 |
|----|-----|----------|
| 使能 | 0x201~0x204 | `FF FF FF FF FF FF FF FC`（上升沿触发）|
| 失能 | 0x201~0x204 | `FF FF FF FF FF FF FF FD`（下降沿触发）|
| 清错 | 0x201~0x204 | `FF FF FF FF FF FF FF FB`（上升沿触发）|
| 速度 | 0x201~0x204 | 4 字节小端 float（每 10 ms 一帧，依次发 4 个电机）|
| 状态 | 0x000（所有）| 8 字节反馈，软件按 ID 分发到对应电机状态区 |

电机状态帧 8 字节布局：

| 字节 | 内容 |
|------|------|
| 0 | ID[3:0] \| ERR[7:4] |
| 1 | POS[15:8] |
| 2 | POS[7:0] |
| 3 | VEL[11:4] |
| 4 | VEL[3:0] \| T[11:8] |
| 5 | T[7:0] |
| 6 | T_MOS |
| 7 | T_Rotor |

电机 ID 1~4 对应状态寄存器 0x0064/0x006E/0x0078/0x0082（间隔 10），每块 10 寄存器，布局为 `ERR + reserved + POS(float32) + VEL(float32) + T(float32) + T_MOS + T_Rotor`。POS/VEL/T 由帧内定点数经 `uint_to_float()` 映射得到，量程分别为 ±12.5 / ±200 / ±10。

> 保持寄存器表总长 **512**（0x0000~0x01FF）。0x0000~0x00BF 与 running 逐地址一致；0x00C0~0x00FF 为保留区；0x0100 起是本工程后加的风机区。

## 7. 风机控制

**PWM 输出（20 kHz，TIM8；`PSC=83 / ARR=99` → 168 MHz / 84 / 100 = 20 kHz）**：

- 风机 1 → PWM5 (TIM8_CH1, PC6)
- 风机 2 → PWM6 (TIM8_CH2, PI6)
- 风机 3 + 风机 4 → PWM7 (TIM8_CH3, PI7)，取两者较大值

**占空比的来源有两条路，都在 `ModbusTask` 里每 5 ms 执行一次**（`modbus_poll()`，**不是 FanTask**）：

| 模式 | 条件 | 输出 |
|---|---|---|
| **手动** | `0x0140 = 0` **或** `0x0141 = 0` | `sync_fan_outputs_from_regs()` —— 直接跟随 `0x0100~0x0103`（截断 0~100）|
| **自动·姿态前馈** | `0x0140 ≠ 0` **且** `0x0141 = 1` | `fan_auto_update()` —— 由 roll/pitch 算出单值，四路同值直写 CCR |
| ⚠️ **自动，但 IMU 未就绪** | 同上，且 `0x015E ≠ 2` | 输出 **θ=0° 档位**（查表 → `DUTY_LUT[0]`；线性 → `DUTY_FLAT`），保证预热期有基础吸附 |

前馈的两套模型与标定见 `标定手册.md` §3（线性，仅 [0°,90°]）与 §8（LUT，覆盖 [0°,180°]）。
当前占空比读 `0x015C`（`AUTO_DUTY`，float32）。

> ⚠️ **`FanTask` 目前不参与输出** —— 它只调 `fan_tick()`，而那个斜坡状态机恒为 `IDLE`
> （`fan_set_target()` / `fan_stop()` 全工程无人调用）。**别被任务名误导。**

**FG 测速**：`PulseTask` 每 10 ms 累计 FG 脉冲，每 500 ms 窗口算 RPM，通过
`pulse_get_freq_hz()` / `pulse_get_rpm()` / `pulse_get_pulse_count()` 读取（`bsp/boards/bsp_pulse.c`）。
⚠️ **PPR 硬编码为 2**、RPM 分辨率约 **60 RPM**，详见 `接口文档.md` §6.2。

## 8. 构建与烧录

构建命令（工程根目录）：

```bash
cmake --preset Debug
cmake --build build/Debug -- -j 8
```

产物：`build/Debug/pwm_snail.{elf,hex,bin}`，当前约 **74 KB Flash / 29 KB RAM**
（`RAM 29848 B (22.77%) / FLASH 75436 B (7.19%)`）。烧录用 ST-Link 即可
（`openocd.cfg` 与 `.vscode/tasks.json` 里有现成的构建 / 烧录 / 调试任务）。

**工具链现状（2026-09-15 起）** —— 三个组件都是独立安装的，**不需要 STM32CubeIDE**：

| 组件 | 位置 | 备注 |
|---|---|---|
| CMake 4.3.3 | `D:\ST\bin\cmake.exe` | ⚠️ **不在 PATH 里**，需用绝对路径或自行加 PATH |
| Ninja 1.13.2 | `D:\stm32\nijia\ninja.exe` | 目录名就是 "nijia"（非笔误）|
| arm-none-eabi-gcc 12.2.1 | `D:\stm32\Arm GNU Toolchain arm-none-eabi\12.2 mpacbti-rel1\bin` | 工具链文件要求它在 PATH 中 |

> ⚠️ 曾出现"ninja 报 `Re-running CMake... CreateProcess failed`"——原因是 `build.ninja` 里
> `RERUN_CMAKE` 边依赖的 CMake 模块文件随 STM32CubeIDE 一起被删了。**装回独立 CMake 后
> `cmake --preset Debug` 会自动重生 `build.ninja`、恢复该边。**

> ⚠️ **不要再用 `pwm_snail.ioc` 重新生成代码。** 该 `.ioc` 停留在移植初期：
> 它仍勾选着已废弃的 `USB_DEVICE`（CDC），重新生成会把 `Src/usb_device.c` 等文件和一整套
> USB 初始化再拉回来；更严重的是**重新生成会覆盖 `main.c` / `gpio.c` / `usart.c` / `tim.c`**，
> 抹掉手改的 **PG0 EXTI 配置**（IMU 采集链路的命门）、**USART6 裸寄存器收发**、
> **TIM10 恒温 PWM** 等 —— 这些都不在 CubeMX 的表达能力之内。
> `.ioc` 现在只作**引脚分配参考**保留，真值以 `Src/` 下的代码为准。
> （同理，`MDK-ARM/` Keil 工程已删除，本项目只用 CMake + GCC 工具链。）

## 9. 当前状态

已完成：

- 4 路 FG 反馈采集（TIM1_CH1~CH4 输入捕获，500 ms 测频）
- 3 路 20 kHz PWM 输出（TIM8_CH1~CH3）
- FreeRTOS 4 任务，1 ms SysTick + 1 ms TIM6 HAL 时基
- RS485 Modbus RTU 从站（03/04/06/10，0x0100 风机 + 0x0000 电机双区）
- 1 Mbps CAN 电机控制（PD0/PD1，AF9）
- 正交全向轮运动学解算（与 `D:\stm32\running\Core\Src\kinematics.c` 逐行一致）
- 通信丢失保护 / NaN 防护 / 500 ms 心跳超时
- 参数区 Flash A/B 双区断电保持，2 s 落盘防抖。
  ⚠️ **2026-09-16 修复**：CRC 的校验范围曾把 `crc` 字段自身也算进去，导致校验恒失败、
  **断电保持从来没有生效过**；现 `crc` 已移到镜像块末尾，`FLASH_RETENTIVE_VERSION = 3`。
  已实机验证：写 `0x0140 = 2` → 复位 → 读回仍为 **2**。
- IMU 六轴姿态解算（BMI088 + Mahony）+ 陀螺零偏/温度闭环。姿态输出到 `0x0150~0x015F`
  （欧拉角/角速度/`0x015C` 前馈占空比/状态/温度）、`0x016E~0x016F`（实时总倾角 θ）、
  **`0x0174~0x017B`（姿态四元数，机体系→世界系，无万向节死锁，做算法优先用它）**
- 风机自动模式：由 roll/pitch 前馈计算占空比。**两套模型**：
  `FAN_MODE=1` 线性（`duty = DUTY_FLAT + SLOPE_GAIN×(1−cosθ)`，参数区 `0x0140~0x014F`，**只适用 [0°,90°]**）；
  `FAN_MODE=2` **查表 LUT**（`0x0160~0x016D`，覆盖 θ ∈ [0°,180°]，**风洞外围爬行用这个**）。
  详见 `标定手册.md` §8。
- **预热期的安全兜底（2026-09-16）**：自动模式下若 IMU 尚未进入 RUNNING，
  输出 **θ=0° 档位**（而非 0），保证预热那几秒有基础吸附、且与接管后输出连续不跳变。
  已实机验证：预热期 `0x015C` = **30.0**。
- **恒温 PID 提前切闭环（2026-09-16）**：温度一到 `目标 − 2℃` 就交 PID，
  不再等"连续确认达标"（原实现那段确认窗口里仍在满功率加热，实测过冲到 49℃）。

剩余项（按后续计划）：

- 上位机 / EBS-P300 / ROS2 联调
- 真实硬件跑直线/自转验证电机方向与 running 一致
- **倾斜角 → 风机占空比标定**。两条路：
  ① 只在 [0°,90°] 内用 → 求 `DUTY_FLAT` / `SLOPE_GAIN`（§3）；
  ② 风洞外围爬行（要覆盖到 180°）→ **查表 LUT 13 点**（§8，**已在固件中实现**，
  出厂值为理论占位值，上真实工况前须标定覆盖）。流程均见 `标定手册.md`。
- ✅ **恒温加热（2026-09-16 已定位并解决）**：**24V 正常供电下约 2 s 升到 45℃ 目标**（实测）。
  早前记录的"满功率只到 ~34℃"是**用 USB 供电**造成的 —— 供电带不动加热回路，
  **不是加热电路的问题**。
  ⚠️ 因此**调试/运行请用 24V 供电（电池或直流电源），不要用 USB 跑加热**；
  否则温度到不了 45℃，预热会一路拖到 20 s 超时兜底才放行。
  （另：温度超过目标后需连续确认 `INS_TEMP_CONFIRM_CNT`=30 次（30×50ms=1.5s）才认达标，
  2026-09-16 由 200 次下调 —— 原值等于白等 10 秒。）

## 10. 上层使用提示（避坑）

- **风机占空比从 0x0000 改到 0x0100 了**。旧版上位机/示例若读 0x0000 寄存器，会拿到电机使能位（不是风机占空比），请同步更新。
- **CAN1 引脚是 PD0/PD1**，不是 PA11/PA12。早期示例若按 PA11/PA12 接线需改线。
- **不要直接写电机速度寄存器 0x000A~0x0011**，会被运动学解算结果覆盖。应只发 0x0014~0x0019（vy/vx/wz）。
- **心跳**：建议上位机每 50 ms 写一次 0x0014~0x0019，即使速度不变也要写，否则 500 ms 后会被强制停车。
