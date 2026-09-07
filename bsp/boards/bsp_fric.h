#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

/* 风机占空比范围（0~100），对应 TIM1 PWM Period=99 */
#define FRIC_DUTY_MIN   0     /* 完全停止 */
#define FRIC_DUTY_START 30    /* 最低启动占空比 30% */
#define FRIC_DUTY_MAX   100   /* 最大吸力 100% */

/* 风机控制状态机 */
typedef enum {
    FAN_STATE_IDLE = 0,    /* EN=0, PWM=0 */
    FAN_STATE_RUN,         /* EN=1, PWM 斜坡跟踪目标占空比 */
    FAN_STATE_STOPPING     /* PWM 斜坡降到 0，再延时 200ms 拉低 EN */
} fan_state_t;

/* 兼容旧 API（直接设置 CCR 值，0~100） */
#define FRIC_OFF  FRIC_DUTY_MIN

extern void fric_off(void);
extern void fric_on(uint16_t cmd);   /* cmd: 0~100 占空比 */

/* 新增：EN 引脚与斜坡控制 */
extern void fric_en_on(void);
extern void fric_en_off(void);
extern void fric_set_duty(uint8_t duty);   /* 立即设置占空比 0~100 */
extern void fan_set_target(uint8_t target_duty);
extern void fan_stop(void);
extern void fan_tick(void);   /* 在 while(1) 主循环里调用，50ms 一次 */
extern uint8_t fan_get_current_duty(void);
extern uint8_t fan_get_target_duty(void);
extern fan_state_t fan_get_state(void);

#endif
