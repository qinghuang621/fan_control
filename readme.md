# STM32 风扇控制项目

## 项目概览

本项目面向 RoboMaster 开发板 C 型，使用 STM32F407 控制多路风机/电机接口：

- PWM1~PWM4：接收四路 FG 转速反馈。
- PWM5~PWM7：输出三路同步的 20 kHz PWM 控制信号。
- USB CDC：提供上位机虚拟串口控制和状态查询。
- 主循环：执行风机状态机、脉冲统计和板载 LED 效果。

## 硬件映射

| 接口 | 定时器通道 | STM32 引脚 | 当前功能 |
| --- | --- | --- | --- |
| PWM1 | TIM1_CH1 | PE9 | FG 输入捕获 |
| PWM2 | TIM1_CH2 | PE11 | FG 输入捕获 |
| PWM3 | TIM1_CH3 | PE13 | FG 输入捕获 |
| PWM4 | TIM1_CH4 | PE14 | FG 输入捕获 |
| PWM5 | TIM8_CH1 | PC6 | 20 kHz PWM 输出 |
| PWM6 | TIM8_CH2 | PI6 | 20 kHz PWM 输出 |
| PWM7 | TIM8_CH3 | PI7 | 20 kHz PWM 输出 |

PWM1~PWM4 使用 STM32 内部上拉。PWM5~PWM7 使用相同的计数器周期和比较值，因此输出相同频率与占空比。

## 软件结构

- `Src/main.c`：系统初始化和主循环。
- `Src/tim.c`：TIM1 输入捕获、TIM8 PWM 输出及 GPIO 复用配置。
- `bsp/boards/bsp_fric.c`：占空比控制和风机启停状态机。
- `bsp/boards/bsp_pulse.c`：四路 FG 中断计数、500 ms 窗口测频和 RPM 计算。
- `Src/usbd_cdc_if.c`：USB CDC 接收命令、输出状态信息。
- `bsp/boards/bsp_led.c`：板载 RGB LED 控制。
- `pwm_snail.ioc`：STM32CubeMX 外设和引脚配置源文件。

## 控制接口

通过 USB 虚拟串口发送以换行结束的命令：

- `START`：以默认启动占空比启动风机。
- `S0`~`S100`：设置目标占空比；`S0` 等同于停止。
- `STOP`：执行斜坡停机。
- `STATUS`：读取当前状态、共同占空比以及 PWM1~PWM4 的频率、RPM 和累计脉冲数。
- `PPRn`：设置每转脉冲数，例如 `PPR2`。
- `HELP`：查看命令列表。

## 构建

使用 ARM GNU Toolchain 和 CMake Presets 构建：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel 4
```

构建完成后会在构建目录生成 ELF、HEX 和 BIN 文件，可使用对应烧录工具下载到开发板。
