#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

/* DWT 周期计数器初始化（微秒级忙等的基础）。C 板 SysTick 归 FreeRTOS，
 * 例程那套基于 SysTick 的 delay_us 不可用，故改用 DWT。 */
extern void bsp_dwt_init(void);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif
