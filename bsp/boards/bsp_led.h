#ifndef BSP_LED_H
#define BSP_LED_H
#include "struct_typedef.h"

/**
 * @brief 初始化板载 RGB LED（TIM5 CH1/2/3 -> PH10/PH11/PH12）
 * @note  完整初始化 TIM5+GPIO，不依赖 CubeMX 生成的 tim.c
 *        调用一次即可，可在 main() 外设初始化阶段调用
 */
extern void led_init(void);

/**
 * @brief LED 状态机周期调用
 * @note  在主循环里周期调用（推荐 5ms 一次），实现彩虹渐变+亮度呼吸
 *        色相 0~360° 约 3.9 秒一圈，亮度 0~255→0 约 1.3 秒一次，完整周期约 5 秒
 */
extern void led_tick(void);

/**
 * @brief 直接显示 aRGB 颜色（手动覆盖）
 * @param aRGB 0xAA RR GG BB，AA=alpha(亮度0~255), RR/GG/BB=0~255
 */
extern void aRGB_led_show(uint32_t aRGB);

#endif
