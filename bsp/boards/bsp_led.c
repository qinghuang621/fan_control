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

/* ============================================================================
 * 彩虹渐变 + 亮度呼吸（led_tick）已移除
 *
 * 原因：它与倾斜指示灯（bsp_led_tilt.c）争夺同一组硬件资源 —— TIM5 的
 * CCR1/2/3 分别驱动 PH10(蓝)/PH11(绿)/PH12(红)，全板只有这一组，后写者
 * 覆盖先写者。两者共存会导致颜色乱闪、呼吸不成形。
 *
 * 例程 661c2-main/18.ins_task 的处理也是二选一（其 freertos.c 里把
 * led_RGB_flow_task 的创建注释掉了，只保留 led_tilt_task）。
 *
 * 本工程选择保留倾斜指示灯：调试期它能直观反映 IMU 是否出数、倾斜方向
 * 是否正确，价值高于纯装饰性的彩虹灯。呼吸语义由倾斜指示的亮度层承载
 * （恒温加热中 1Hz 慢呼吸）。
 *
 * 如需回退彩虹灯：恢复下方 led_tick 实现，并让 LedTask 改调它、同时
 * 停用 led_tilt_update()，二者不可同时运行。
 * ========================================================================= */

/* ============================================================================
 * 致命错误报警：红光 SOS 式闪烁
 *
 * 用途：Error_Handler() 被调用时点亮红色，替代原来的"什么都不做"。
 * 原来 Error_Handler 是空实现，SPI/DMA/CAN 等外设初始化失败会静默停在
 * 空函数里，现象是"板子像没跑起来"，极易误判为硬件故障或不认为是软件问题。
 *
 * 实现要点（为什么不能直接调 aRGB_led_show）：
 *   1. Error_Handler 可能在 led_init() 之前被调用（如 MX_GPIO_Init 失败），
 *      此时 TIM5 还没配置，写 CCR 无效。所以这里**不依赖 led_init()**，
 *      自己配置 PH10/11/12 为普通推挽输出，用软件延时闪灯。
 *   2. 本函数**不返回**（保持原 Error_Handler 的语义：出错即停）。
 *   3. 这里用忙等而非 vTaskDelay：本函数可能在调度器启动前被调用。
 *
 * LED 极性：C 板 LED 为高电平点亮（与 gpio.c 里 LED_R/G/B_Pin 初始 SET 一致）。
 * ========================================================================= */
void led_fatal_blink(void)
{
    GPIO_InitTypeDef gpio = {0};
    volatile uint32_t i;

    /* 自己开时钟 + 配引脚，不依赖 led_init() 是否已执行 */
    __HAL_RCC_GPIOH_CLK_ENABLE();

    gpio.Pin   = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &gpio);

    /* 无限闪烁：红亮 300ms / 灭 300ms，肉眼一眼可辨 */
    for (;;)
    {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);    /* 红亮 */
        for (i = 0; i < 2000000U; i++) { __NOP(); }

        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);  /* 红灭 */
        for (i = 0; i < 2000000U; i++) { __NOP(); }
    }
}

