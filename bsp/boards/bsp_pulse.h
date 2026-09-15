#ifndef BSP_PULSE_H
#define BSP_PULSE_H

#include <stdint.h>

/* 脉冲输入测量（TIM1_CH1~CH4 = C板 PWM1~PWM4）—— 四路 FG 测速
 *
 * 物理映射：
 *   - PWM1 / TIM1_CH1 -> 风机编号 1 的 FG 反馈
 *   - PWM2 / TIM1_CH2 -> 风机编号 2 的 FG 反馈
 *   - PWM3 / TIM1_CH3 -> 风机编号 3 的 FG 反馈
 *   - PWM4 / TIM1_CH4 -> 风机编号 4 的 FG 反馈
 *
 * 用法（本工程由 PulseTask 驱动，周期 10ms）：
 *   pulse_capture_init();                       // main 初始化时调用一次
 *   pulse_poll();                               // PulseTask 每 10ms 调一次
 *   uint32_t freq = pulse_get_freq_hz(1);       // 风机 1 脉冲频率 Hz
 *   uint32_t rpm  = pulse_get_rpm(1, 2);        // 风机 1 转速 RPM（第 2 参为 PPR，现传 2）
 *   uint32_t cnt  = pulse_get_pulse_count(1);   // 风机 1 累计脉冲总数
 *
 * 硬件：白线信号线接 PWM1~PWM4 排针信号脚，GND 与开发板共地。
 * 注意：输入脚只耐受 3.3V，若信号是 5V 需加分压/电平转换。
 */

void     pulse_capture_init(void);
void     pulse_poll(void);                    /* 周期调用（PulseTask 10ms），内部 500ms 窗口测频 */

uint32_t pulse_get_freq_hz(uint8_t channel);       /* channel: 1~4 */
uint32_t pulse_get_rpm(uint8_t channel, uint32_t ppr);
uint32_t pulse_get_pulse_count(uint8_t channel);

#endif
