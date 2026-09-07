#include "bsp_led.h"
#include "stm32f4xx_hal.h"

/* TIM5 句柄（本模块独立维护，不与 CubeMX 生成的 tim.c 共享） */
static TIM_HandleTypeDef s_htim5;

/**
 * @brief 完整初始化 TIM5 + PH10/PH11/PH12 引脚
 *        TIM5 CH1=PH10(蓝)  CH2=PH11(绿)  CH3=PH12(红)
 *        TIM5 时钟 = APB1*2 = 84MHz，Prescaler=0, Period=65535 -> PWM 1.28kHz
 */
void led_init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. 启用 TIM5 和 GPIOH 时钟 */
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* 2. 配置 PH10/PH11/PH12 为 AF2（TIM5）推挽复用 */
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* 3. 配置 TIM5 时基 */
    s_htim5.Instance               = TIM5;
    s_htim5.Init.Prescaler         = 0;
    s_htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim5.Init.Period            = 65535;
    s_htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_htim5) != HAL_OK) return;

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&s_htim5, &sClockSourceConfig);

    if (HAL_TIM_PWM_Init(&s_htim5) != HAL_OK) return;

    /* 4. 配置 3 路 PWM 通道（初始占空比 0） */
    sConfigOC.OCMode      = TIM_OCMODE_PWM1;
    sConfigOC.Pulse       = 0;
    sConfigOC.OCPolarity  = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&s_htim5, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&s_htim5, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&s_htim5, &sConfigOC, TIM_CHANNEL_3);

    /* 5. 启动 TIM5 和 3 路 PWM */
    HAL_TIM_Base_Start(&s_htim5);
    HAL_TIM_PWM_Start(&s_htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&s_htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&s_htim5, TIM_CHANNEL_3);
}

/**
 * @brief 显示 aRGB 颜色
 * @param aRGB 0xAA RR GG BB
 */
void aRGB_led_show(uint32_t aRGB)
{
    uint8_t  alpha = (uint8_t)((aRGB & 0xFF000000U) >> 24);
    uint16_t red   = (uint16_t)(((aRGB & 0x00FF0000U) >> 16) * alpha);
    uint16_t green = (uint16_t)(((aRGB & 0x0000FF00U) >>  8) * alpha);
    uint16_t blue  = (uint16_t)(((aRGB & 0x000000FFU) >>  0) * alpha);

    /* 8bit × 8bit = 16bit，正好填入 Period=65535 的 CCR */
    __HAL_TIM_SetCompare(&s_htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&s_htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&s_htim5, TIM_CHANNEL_3, red);
}

/**
 * @brief 彩虹渐变 + 亮度呼吸（每 5ms 调用一次效果最佳）
 *        色相 0~1535 共 6 段 × 256，hue+=2 -> 768 步 × 5ms ≈ 3.9 秒一圈
 *        亮度 0~255 -> 256 步 × 5ms ≈ 1.3 秒呼吸一次
 *        完整周期约 5 秒
 */
void led_tick(void)
{
    static uint16_t hue   = 0;   /* 色相 0~1535 */
    static uint8_t  alpha = 0;   /* 亮度 0~255 */
    static int8_t   dir   = 1;   /* 亮度方向 +1/-1 */
    uint8_t r, g, b;

    /* HSV 色相 -> RGB (饱和度 100%, 明度由 alpha 控制) */
    if      (hue < 256)  { r = 255;            g = (uint8_t)hue;        b = 0;   }  /* R -> Y */
    else if (hue < 512)  { r = (uint8_t)(511 - hue); g = 255;           b = 0;   }  /* Y -> G */
    else if (hue < 768)  { r = 0;              g = 255;           b = (uint8_t)(hue - 512); }  /* G -> C */
    else if (hue < 1024) { r = 0;              g = (uint8_t)(1023 - hue); b = 255; }  /* C -> B */
    else if (hue < 1280) { r = (uint8_t)(hue - 1024); g = 0;            b = 255; }  /* B -> M */
    else                 { r = 255;           g = 0;             b = (uint8_t)(1535 - hue); }  /* M -> R */

    /* 打包 aRGB: AA RR GG BB */
    aRGB_led_show(((uint32_t)alpha << 24) |
                  ((uint32_t)r << 16)     |
                  ((uint32_t)g << 8)      |
                  ((uint32_t)b));

    /* 步进 */
    hue += 2;                    /* 色相每步 +2 */
    if (hue >= 1536) hue = 0;

    alpha = (uint8_t)(alpha + dir);
    if (alpha == 255) dir = -1;
    if (alpha == 0)   dir =  1;
}
