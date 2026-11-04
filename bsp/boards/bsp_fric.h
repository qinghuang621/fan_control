#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

/* 风机占空比范围（0~100），对应 TIM8 PWM Period=99 */
#define FRIC_DUTY_MIN   0     /* 完全停止 */
#define FRIC_DUTY_MAX   100   /* 最大吸力 100% */

extern void fric_off(void);

/* 按 PWM 通道独立设置占空比，通道 1~3 对应 C 板 PWM5~PWM7
 * （PWM7 上挂风机 3 与风机 4，两路共用同一占空比） */
extern void fric_set_channel_duty(uint8_t channel, uint8_t duty);
extern uint8_t fric_get_channel_duty(uint8_t channel);

#endif
