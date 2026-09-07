#include "bsp_fric.h"
#include "main.h"

extern TIM_HandleTypeDef htim8;

/* 风机控制状态机变量 */
static fan_state_t s_fan_state = FAN_STATE_IDLE;
static uint8_t s_current_duty = 0;     /* 当前实际占空比 */
static uint8_t s_target_duty  = 0;    /* 目标占空比 */
static uint32_t s_last_tick   = 0;    /* 上次斜坡更新时间 */
static uint32_t s_stop_timer  = 0;   /* 停机延时计时 */

#define RAMP_INTERVAL_MS  50   /* 每 50ms 走一步 */
#define RAMP_STEP        1     /* 步进 1%，0->100% 约 5s */
#define STOP_DELAY_MS    200   /* PWM 归零后保持 200ms 再断 EN */

/* 内部：写 TIM8_CH1 的 CCR (0~100)，对应 PC6 = C板 PWM5 接口 */
static void fric_apply_ccr(uint16_t cmd)
{
    /* TIM8 Period=99，CCR 范围 0~100，cmd 直接等于占空比百分比 */
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, cmd);
}

/* 旧 API：关闭摩擦轮/风机（占空比 0） */
void fric_off(void)
{
    fric_apply_ccr(FRIC_DUTY_MIN);
}

/* 旧 API：设置占空比 cmd (0~100) */
void fric_on(uint16_t cmd)
{
    if (cmd > FRIC_DUTY_MAX) cmd = FRIC_DUTY_MAX;
    fric_apply_ccr(cmd);
}

/* 新增：立即设置占空比 0~100 */
void fric_set_duty(uint8_t duty)
{
    if (duty > FRIC_DUTY_MAX) duty = FRIC_DUTY_MAX;
    s_current_duty = duty;
    fric_apply_ccr(duty);
}

/* EN 引脚控制（已弃用：风机直接由 PWM 控制，无需 EN 使能脚） */
void fric_en_on(void)
{
    /* no-op */
}

void fric_en_off(void)
{
    /* no-op */
}

/* 设置目标占空比，启动风机（如未运行） */
void fan_set_target(uint8_t target_duty)
{
    if (target_duty > FRIC_DUTY_MAX) target_duty = FRIC_DUTY_MAX;
    s_target_duty = target_duty;

    if (s_fan_state == FAN_STATE_IDLE)
    {
        /* 启动：先拉高 EN，再开始斜坡（EN 信号纳秒级建立，无需延时） */
        fric_en_on();
        s_fan_state = FAN_STATE_RUN;
        s_last_tick = HAL_GetTick();
    }
    else if (s_fan_state == FAN_STATE_STOPPING)
    {
        /* 中途取消停机 */
        s_fan_state = FAN_STATE_RUN;
        s_last_tick = HAL_GetTick();
    }
}

/* 请求停机：进入斜坡降速状态 */
void fan_stop(void)
{
    if (s_fan_state == FAN_STATE_RUN)
    {
        s_target_duty = 0;
        s_fan_state = FAN_STATE_STOPPING;
        s_last_tick = HAL_GetTick();
    }
}

/* 主循环周期调用，实现斜坡调速 + 停机延时断电 */
void fan_tick(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_fan_state)
    {
        case FAN_STATE_IDLE:
            /* 空闲：保持 PWM=0, EN=0 */
            break;

        case FAN_STATE_RUN:
            if (now - s_last_tick >= RAMP_INTERVAL_MS)
            {
                s_last_tick = now;
                if (s_current_duty < s_target_duty)
                {
                    uint8_t next = s_current_duty + RAMP_STEP;
                    if (next > s_target_duty) next = s_target_duty;
                    fric_set_duty(next);
                }
                else if (s_current_duty > s_target_duty)
                {
                    uint8_t next = (s_current_duty >= RAMP_STEP) ? (s_current_duty - RAMP_STEP) : 0;
                    if (next < s_target_duty) next = s_target_duty;
                    fric_set_duty(next);
                }
                /* 已达目标占空比，保持 */
            }
            break;

        case FAN_STATE_STOPPING:
            if (now - s_last_tick >= RAMP_INTERVAL_MS)
            {
                s_last_tick = now;
                if (s_current_duty > 0)
                {
                    uint8_t next = (s_current_duty >= RAMP_STEP) ? (s_current_duty - RAMP_STEP) : 0;
                    fric_set_duty(next);
                }
                else
                {
                    /* PWM 已归零，开始 200ms 倒计时 */
                    if (s_stop_timer == 0)
                    {
                        s_stop_timer = now;
                    }
                    else if (now - s_stop_timer >= STOP_DELAY_MS)
                    {
                        fric_en_off();
                        s_stop_timer = 0;
                        s_fan_state = FAN_STATE_IDLE;
                    }
                }
            }
            break;

        default:
            s_fan_state = FAN_STATE_IDLE;
            break;
    }
}

uint8_t fan_get_current_duty(void) { return s_current_duty; }
uint8_t fan_get_target_duty(void)  { return s_target_duty;  }
fan_state_t fan_get_state(void)    { return s_fan_state;    }
