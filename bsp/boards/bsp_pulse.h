#ifndef BSP_PULSE_H
#define BSP_PULSE_H

#include <stdint.h>

/* 脉冲输入测量（TIM1_CH1~CH4 = C板 PWM1~PWM4）—— 四路 FG 测速
 *
 * 用法：
 *   pulse_capture_init();                       // main 初始化时调用一次
 *   while(1) { pulse_poll(); ... }              // 主循环周期调用（5ms 节拍）
 *   uint32_t freq = pulse_get_freq_hz(1);       // PWM1 脉冲频率 Hz
 *   uint32_t rpm  = pulse_get_rpm(1, ppr);      // PWM1 转速 RPM
 *   uint32_t cnt  = pulse_get_pulse_count(1);   // PWM1 累计脉冲总数
 *
 * 硬件：白线信号线接 PWM1~PWM4 排针信号脚，GND 与开发板共地。
 * 注意：输入脚只耐受 3.3V，若信号是 5V 需加分压/电平转换。
 */

void     pulse_capture_init(void);
void     pulse_poll(void);                    /* 主循环周期调用，500ms 窗口测频 */

uint8_t  pulse_is_valid(void);        /* 是否已完成过至少一次窗口测量 */
uint32_t pulse_get_freq_hz(uint8_t channel);       /* channel: 1~4 */
uint32_t pulse_get_rpm(uint8_t channel, uint32_t ppr);
uint32_t pulse_get_pulse_count(uint8_t channel);

#endif
