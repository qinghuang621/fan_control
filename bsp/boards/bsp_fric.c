#include "bsp_fric.h"
#include "main.h"

extern TIM_HandleTypeDef htim8;

/* 风机 PWM 通道占空比缓存（0~100），通道 1~3 对应 TIM8_CH1~CH3（C 板 PWM5~PWM7）。
 * 物理连接约定：
 *   - PWM5 -> 风机 1
 *   - PWM6 -> 风机 2
 *   - PWM7 -> 风机 3 与风机 4 共用，两个风机共用同一个占空比输出
 *
 * ⚠️ 历史说明（2026-11-04）：本文件原有一套「斜坡软启动状态机」
 * （fan_set_target / fan_stop / fan_tick + FanTask 每 5ms 轮询）。
 * 但那套逻辑的入口 fan_set_target() 全仓从未被调用，状态恒为 IDLE，
 * 软启动从未生效；实际输出一直由本文件的 fric_set_channel_duty() 直接写 CCR。
 * 该状态机与 FanTask 已一并删除（回收 1024 B 任务栈）。
 *
 * 若将来需要软启动，应把斜坡做在 fric_set_channel_duty() 内部：
 * 四路各自独立、自动/手动两条路径都能覆盖，不要再恢复「3 路同值 + 任务轮询」的旧设计。 */
static uint8_t s_channel_duty[3] = {0, 0, 0};

void fric_set_channel_duty(uint8_t channel, uint8_t duty)
{
    if (channel < 1U || channel > 3U)
    {
        return;
    }
    if (duty > FRIC_DUTY_MAX)
    {
        duty = FRIC_DUTY_MAX;
    }
    s_channel_duty[channel - 1U] = duty;

    switch (channel)
    {
        case 1U:
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, duty);
            break;
        case 2U:
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, duty);
            break;
        case 3U:
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, duty);
            break;
        default:
            break;
    }
}

uint8_t fric_get_channel_duty(uint8_t channel)
{
    if (channel < 1U || channel > 3U)
    {
        return 0U;
    }
    return s_channel_duty[channel - 1U];
}

/* 关闭全部风机（占空比归零）。上电时用于确立安全状态。 */
void fric_off(void)
{
    fric_set_channel_duty(1U, FRIC_DUTY_MIN);
    fric_set_channel_duty(2U, FRIC_DUTY_MIN);
    fric_set_channel_duty(3U, FRIC_DUTY_MIN);
}
