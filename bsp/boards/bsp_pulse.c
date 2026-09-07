#include "bsp_pulse.h"
#include "main.h"
#include "tim.h"

/* TIM1_CH1~CH4 输入捕获（PWM1~PWM4）—— 四路 FG 测速脉冲计数（窗口法）
 *
 * 设计要点（防御性，避免旧版跑飞问题）：
 *  1. 只使能 CC1~CC4 捕获中断，不使用 TIM1 更新中断
 *  2. 中断回调只做对应通道脉冲计数 +1，不做任何计算
 *  3. 频率用主循环 500ms 窗口内计数差计算，无需溢出补偿
 *  4. CC2 加输入滤波，抗信号毛刺
 *
 * RPM = 频率(Hz) × 60 ÷ 每转脉冲数(PPR)，PPR 由上位机 PPRn 命令设定（存 usbd_cdc_if.c）
 */

static volatile uint32_t s_pulse_count[4];
static volatile uint32_t s_freq_hz[4];
static volatile uint8_t  s_have_freq[4];

void pulse_capture_init(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        s_pulse_count[i] = 0;
        s_freq_hz[i] = 0;
        s_have_freq[i] = 0;
    }

    /* TIM1 已由 CubeMX MX_TIM1_Init() 配置（IC CH2 + GPIO）。
     * 这里用寄存器级微调：精确 1MHz 计数 + CC2 滤波，不重建 HAL 状态。 */
    TIM1->CR1   &= ~TIM_CR1_CEN;                     /* 先停计数 */
    TIM1->PSC    = 167;                              /* 168MHz/168 = 1MHz, 1 tick = 1us */
    TIM1->ARR    = 0xFFFF;                           /* 满量程（窗口法不依赖溢出） */
    /* CC1~CC4: 直连输入、无分频、数字滤波，均捕获上升沿 */
    TIM1->CCMR1 = (3U << 4) | (1U << 0) | (3U << 12) | (1U << 8);
    TIM1->CCMR2 = (3U << 4) | (1U << 0) | (3U << 12) | (1U << 8);
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                  TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->SR     = 0;                                /* 清残留标志 */
    TIM1->DIER   = TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                   TIM_DIER_CC3IE | TIM_DIER_CC4IE;
    TIM1->CR1   |= TIM_CR1_CEN;                      /* 启动计数 */

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 10, 0);       /* 低于 USB，不干扰枚举 */
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/* CC2 捕获中断：HAL_TIM_IRQHandler 清标志后回调到这里，只计数 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        uint8_t channel = 0;
        if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_1) channel = 1;
        else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_2) channel = 2;
        else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_3) channel = 3;
        else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_4) channel = 4;
        if (channel != 0)
        {
            s_pulse_count[channel - 1]++;
        }
    }
}

/* 主循环周期调用（5ms 节拍即可），内部按 500ms 窗口计算频率 */
void pulse_poll(void)
{
    static uint32_t last_cnt[4] = {0};
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if (last_tick == 0)
    {
        last_tick = now;
        for (uint8_t i = 0; i < 4; i++) last_cnt[i] = s_pulse_count[i];
        return;
    }
    if (now - last_tick >= 500)
    {
        uint32_t delta_t = now - last_tick;
        for (uint8_t i = 0; i < 4; i++)
        {
            uint32_t delta_c = s_pulse_count[i] - last_cnt[i];
            s_freq_hz[i] = (delta_c * 1000u) / delta_t;
            s_have_freq[i] = 1;
            last_cnt[i] = s_pulse_count[i];
        }
        last_tick = now;
    }
}

uint8_t pulse_is_valid(void)
{
    return s_have_freq[0] && s_have_freq[1] && s_have_freq[2] && s_have_freq[3];
}

uint32_t pulse_get_freq_hz(uint8_t channel)
{
    return (channel >= 1 && channel <= 4) ? s_freq_hz[channel - 1] : 0;
}

uint32_t pulse_get_pulse_count(uint8_t channel)
{
    return (channel >= 1 && channel <= 4) ? s_pulse_count[channel - 1] : 0;
}

uint32_t pulse_get_rpm(uint8_t channel, uint32_t ppr)
{
    if (ppr == 0) ppr = 1;
    return (pulse_get_freq_hz(channel) * 60u) / ppr;
}
