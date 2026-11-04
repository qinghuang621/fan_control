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
 * @note   手动排障工具，代码中无自动调用点。怀疑加热不工作时调用它拉满 PWM：
 *         温度仍不升 ⇒ 硬件（加热膜/供电/MOS）问题，与 PID 无关。
 */
void imu_pwm_force_full(void);

/* 2026-11-04 停用：无调用，且与 s_heater_pwm / 寄存器 0x014F 重复。
 * 恢复时同步改 bsp_imu_pwm.c 里对应的 #if 0。 */
#if 0
uint16_t imu_pwm_get_last(void);
#endif

/**
 * @brief  兜底启动：若 TIM10 尚未启动则在此配置并启动
 * @note   正常流程下 TIM10 已由 MX_TIM10_Init 初始化，本函数用于容错。
 */
void imu_pwm_ensure_started(void);

#endif
