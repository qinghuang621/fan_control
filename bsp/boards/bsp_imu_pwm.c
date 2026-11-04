#include "bsp_imu_pwm.h"
#include "main.h"
#include "tim.h"

/* 2026-11-04 停用：g_heater_pwm_last / imu_pwm_get_last()
 *
 * 原用途：把最近一次写入的加热 PWM 值缓存下来，供上层诊断读取。
 * 为什么冗余：加热 PWM 已有一条在用的上报路径，不需要这份拷贝 ——
 *     ins_task.c   s_heater_pwm = PID.out   →  imu_pwm_set(s_heater_pwm)
 *     ins_get_heater_pwm() → bsp_modbus.c:1946 → 寄存器 0x014F
 *   s_heater_pwm 在调用 imu_pwm_set() **之前**就已存好，所以本变量是同一份
 *   数据的第二个副本，且全仓无读者（唯一读者 imu_pwm_get_last 也无人调用）。
 * 恢复方法：把下面两处 #if 0 改 #if 1，并恢复 imu_pwm_set() 里的赋值。 */
#if 0
volatile uint16_t g_heater_pwm_last = 0;
#endif

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
}

/**
 * @brief  强制满输出，用于硬件连通性自检
 * @note   手动排障工具：怀疑"加热不工作"时调用本函数把 PWM 拉满，
 *         若温度仍不上升 ⇒ 加热膜/供电/MOS 等硬件问题，与 PID 无关。
 *         当前代码里无自动调用点，需要时在调试处临时调用即可。
 */
void imu_pwm_force_full(void)
{
    imu_pwm_set(MPU6500_TEMP_PWM_MAX);
}

#if 0
uint16_t imu_pwm_get_last(void)
{
    return g_heater_pwm_last;
}
#endif /* 停用：无调用，且与 s_heater_pwm 重复 */

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
