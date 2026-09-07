#include "bsp_pulse.h"
#include "main.h"
#include "tim.h"

/* TIM1_CH2 输入捕获（PE11 = C板 PWM2）—— 风机 FG 测速脉冲计数（窗口法）
 *
 * 设计要点（防御性，避免旧版跑飞问题）：
 *  1. 只使能 CC2 捕获中断，不使用 TIM1 更新中断（不碰 TIM1_UP_TIM10_IRQn）
 *  2. 中断回调只做脉冲计数 +1，不做任何计算
 *  3. 频率用主循环 500ms 窗口内计数差计算，无需溢出补偿
 *  4. CC2 加输入滤波，抗信号毛刺
 *
 * RPM = 频率(Hz) × 60 ÷ 每转脉冲数(PPR)，PPR 由上位机 PPRn 命令设定（存 usbd_cdc_if.c）
 */

static volatile uint32_t s_pulse_count;   /* 累计脉冲总数（中断里累加） */
static volatile uint32_t s_freq_hz;       /* 最近窗口测得的频率 */
static volatile uint8_t  s_have_freq;     /* 是否已完成过至少一次窗口测量 */

void pulse_capture_init(void)
{
    s_pulse_count = 0;
    s_freq_hz     = 0;
    s_have_freq   = 0;

    /* TIM1 已由 CubeMX MX_TIM1_Init() 配置（IC CH2 + GPIO）。
     * 这里用寄存器级微调：精确 1MHz 计数 + CC2 滤波，不重建 HAL 状态。 */
    TIM1->CR1   &= ~TIM_CR1_CEN;                     /* 先停计数 */
    TIM1->PSC    = 167;                              /* 168MHz/168 = 1MHz, 1 tick = 1us */
    TIM1->ARR    = 0xFFFF;                           /* 满量程（窗口法不依赖溢出） */
    /* IC2: CC2S=01(TI2) | IC2PSC=00 | IC2F=0011(8 样本滤波抗毛刺) */
    TIM1->CCMR1  = (3U << 12) | (1U << 8);
    TIM1->CCER  |= TIM_CCER_CC2E;                    /* 上升沿捕获（CC2P=0） */
    TIM1->SR     = 0;                                /* 清残留标志 */
    TIM1->DIER   = TIM_DIER_CC2IE;                   /* 只开 CC2 中断，绝不开 UIE */
    TIM1->CR1   |= TIM_CR1_CEN;                      /* 启动计数 */

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 10, 0);       /* 低于 USB，不干扰枚举 */
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/* CC2 捕获中断：HAL_TIM_IRQHandler 清标志后回调到这里，只计数 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 && (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_2))
    {
        (void)TIM1->CCR2;    /* 读 CCR2 释放锁存 */
        s_pulse_count++;
    }
}

/* 主循环周期调用（5ms 节拍即可），内部按 500ms 窗口计算频率 */
void pulse_poll(void)
{
    static uint32_t last_cnt  = 0;
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if (last_tick == 0)
    {
        last_tick = now;
        last_cnt  = s_pulse_count;
        return;
    }
    if (now - last_tick >= 500)
    {
        uint32_t delta_t = now - last_tick;
        uint32_t delta_c = s_pulse_count - last_cnt;
        s_freq_hz   = (delta_c * 1000u) / delta_t;   /* 个/秒 */
        s_have_freq = 1;
        last_cnt  = s_pulse_count;
        last_tick = now;
    }
}

uint8_t  pulse_is_valid(void)        { return s_have_freq; }
uint32_t pulse_get_freq_hz(void)     { return s_freq_hz; }
uint32_t pulse_get_pulse_count(void) { return s_pulse_count; }

uint32_t pulse_get_rpm(uint32_t ppr)
{
    if (ppr == 0) ppr = 1;
    return (s_freq_hz * 60u) / ppr;
}
