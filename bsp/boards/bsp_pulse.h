#ifndef BSP_PULSE_H
#define BSP_PULSE_H

#include <stdint.h>

/* 脉冲输入测量（TIM1_CH2 = PE11 = C板 PWM2 接口）—— 风机 FG 测速
 *
 * 用法：
 *   pulse_capture_init();                       // main 初始化时调用一次
 *   while(1) { pulse_poll(); ... }              // 主循环周期调用（5ms 节拍）
 *   uint32_t freq = pulse_get_freq_hz();        // 脉冲频率 Hz（窗口计数法）
 *   uint32_t rpm  = pulse_get_rpm(ppr);         // 转速 RPM = 频率×60/每转脉冲数
 *   uint32_t cnt  = pulse_get_pulse_count();    // 累计脉冲总数（上电起）
 *
 * 硬件：白线信号线接 PWM2 排针信号脚(PE11)，GND 与开发板共地。
 * 注意：PE11 只耐受 3.3V，若信号是 5V 需加分压/电平转换。
 */

void     pulse_capture_init(void);
void     pulse_poll(void);                    /* 主循环周期调用，500ms 窗口测频 */

uint8_t  pulse_is_valid(void);        /* 是否已完成过至少一次窗口测量 */
uint32_t pulse_get_freq_hz(void);     /* 脉冲频率 Hz（0 表示无信号）= 每秒脉冲个数 */
uint32_t pulse_get_rpm(uint32_t ppr); /* 转速 RPM = 频率×60/每圈脉冲数 */
uint32_t pulse_get_pulse_count(void); /* 累计脉冲总数（上电起） */

#endif
