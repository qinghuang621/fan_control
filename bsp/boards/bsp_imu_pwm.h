#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H

#include "struct_typedef.h"

/* BMI088 恒温加热 PWM 上限（TIM10 Period = 4999，故最大占空比 100% 对应 4999） */
#define MPU6500_TEMP_PWM_MAX 5000

/**
 * @brief  设置加热电阻 PWM 占空比（自动 clamp 到 MPU6500_TEMP_PWM_MAX 以内）
 */
void imu_pwm_set(uint16_t pwm);

/**
 * @brief  满占空比输出，用于硬件自检/诊断
 */
void imu_pwm_force_full(void);

/**
 * @brief  取最近一次写入的 PWM 值
 */
uint16_t imu_pwm_get_last(void);

/**
 * @brief  兜底启动：若 TIM10 尚未启动则在此配置并启动
 * @note   正常流程下 TIM10 已由 MX_TIM10_Init 初始化，本函数用于容错。
 */
void imu_pwm_ensure_started(void);

#endif
