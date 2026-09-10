#include "bsp_imu_pwm.h"
#include "main.h"
#include "tim.h"

/* 最近一次写入的加热 PWM 值，供上层诊断/上报使用 */
volatile uint16_t g_heater_pwm_last = 0;

/**
 * @brief  设置加热 PWM 占空比
 * @param  pwm 目标比较值，0 ~ MPU6500_TEMP_PWM_MAX
 * @note   写入前做上限钳位。TIM10 的 Period 为 4999，比较值 4999 即 100% 占空比。
 */
void imu_pwm_set(uint16_t pwm)
{
    if (pwm > MPU6500_TEMP_PWM_MAX)
    {
        pwm = MPU6500_TEMP_PWM_MAX;
    }

    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm);
    g_heater_pwm_last = pwm;
}

/**
 * @brief  强制满输出，仅用于硬件连通性自检
 */
void imu_pwm_force_full(void)
{
    imu_pwm_set(MPU6500_TEMP_PWM_MAX);
}

uint16_t imu_pwm_get_last(void)
{
    return g_heater_pwm_last;
}

/**
 * @brief  确保 TIM10 的 PWM 输出处于启动状态
 * @note   即便 MX_TIM10_Init 已完成配置，重复启动也安全（HAL 会重设并使能）。
 *         这里同时负责 PF6 的复用配置兜底，防止 MspPostInit 未生效的极端情况。
 */
void imu_pwm_ensure_started(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}
