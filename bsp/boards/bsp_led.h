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
 * @brief 彩虹渐变 + 亮度呼吸
 * @note  ⚠️ 已移除。它与倾斜指示灯（bsp_led_tilt.c）争夺同一组 TIM5
 *        CCR1/2/3，二者只能二选一；本工程保留倾斜指示灯。
 *        实现与回退说明见 bsp_led.c 中的注释块。
 */
/* extern void led_tick(void); */

/**
 * @brief 直接显示 aRGB 颜色（手动覆盖）
 * @param aRGB 0xAA RR GG BB，AA=alpha(亮度0~255), RR/GG/BB=0~255
 */
extern void aRGB_led_show(uint32_t aRGB);

/**
 * @brief 致命错误报警：红光无限闪烁，**不返回**
 * @note  供 Error_Handler() 调用。不依赖 led_init()，自己配置 PH10/11/12
 *        为推挽输出后用忙等闪灯（可能在调度器启动前被调用）。
 */
extern void led_fatal_blink(void);

#endif
