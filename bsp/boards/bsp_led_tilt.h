#ifndef BSP_LED_TILT_H
#define BSP_LED_TILT_H

#include "struct_typedef.h"

/* ============================ 刷新与阈值参数 ============================ */

/* 实际刷新周期(ms)：50Hz，肉眼流畅且几乎不占 CPU。
 * 本模块由 LedTask 以 5ms 周期调用，内部按此值累加限流。 */
#define LED_TILT_PERIOD_MS        20U

/* 调用者的调用周期(ms)，必须与实际调用间隔一致（LedTask 是 5ms） */
#define LED_TILT_TASK_CALL_PERIOD_MS  5U

/* 死区：|tilt| < 0.5° 显示暗绿 "OK"，避免水平静止时颜色乱跳 */
#define LED_TILT_DEAD_ZONE_DEG    0.5f

/* 轻倾斜上限：|tilt| >= 15° 才用满饱和度 */
#define LED_TILT_OK_DEG           15.0f

/* 告警阈值：|tilt| >= 30° 色相不变、亮度 5Hz 快闪 */
#define LED_TILT_WARN_DEG         30.0f

/* 饱和度饱和点：|tilt| >= 45° 视为饱和 */
#define LED_TILT_FULL_SAT_DEG     45.0f

/* ============================ 对外接口 ============================ */

/**
 * @brief  周期调用，刷新倾斜指示灯（建议 LedTask 里 5ms 调一次）
 * @note   内部按 LED_TILT_PERIOD_MS 限流到 50Hz 实际刷新。
 *         色相 = 倾斜方向，饱和度 = 倾斜幅度，亮度 = 系统状态。
 *         状态着色：
 *           OFFLINE  暗红慢呼吸（等待中）
 *           WARMUP   青色慢呼吸（恒温加热中）
 *           RUNNING  姿态色常亮
 *           ERROR    红色常亮（IMU 故障）
 * @note   ⚠️ 本模块与彩虹呼吸灯 bsp_led.c 的 led_tick() 互斥：
 *         两者都写 TIM5 CCR1/2/3，同一时刻只能有一个在运行。
 */
extern void led_tilt_update(void);

#endif
