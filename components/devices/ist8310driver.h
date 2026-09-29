/**
 * @file    ist8310driver.h
 * @brief   IST8310 磁力计驱动，DJI 18.ins_task 例程移植。
 *          WHO_AM_I=0x10，灵敏度 0.3 uT/LSB。
 */

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR    0x00
#define IST8310_NO_SENSOR   0x40

typedef struct
{
    uint8_t status;
    fp32 mag[3];
} ist8310_real_data_t;

extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *data);
/* 【临时诊断 2026-09-23】返回 1=成功 / 0=失败；raw 输出原始 16 位计数（不乘 0.3）。
 * 原为 `void ist8310_read_mag(fp32 mag[3])`，且内部 buf 未初始化、不检查读失败。 */
extern uint8_t ist8310_read_mag(fp32 mag[3], int16_t raw[3]);

#endif
