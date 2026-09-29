/**
 * @file       IST8310Middleware.c/h
 * @brief      IST8310 磁力计通信中间层，基于硬件 I2C3（PA8/PC9）。
 *             接口风格对齐 BMI088Middleware。
 * @note       DJI 18.ins_task 例程同款（hi2c3 硬件 I2C，非软 I2C）。
 */

#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "struct_typedef.h"

/* I2C 8-bit 地址：0x0E 左移 1 位 = 0x1C */
#define IST8310_IIC_ADDRESS (0x0E << 1)

extern void ist8310_GPIO_init(void);
extern void ist8310_com_init(void);
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
/* 【临时诊断 2026-09-23】返回 1=成功 / 0=失败。原为 void，吞掉了 HAL 状态，
 * 导致读失败时调用方拿未初始化的 buf 当数据解析（"几千 μT"假读数的根因之一）。 */
extern uint8_t ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void);
extern void ist8310_RST_L(void);

/* 【临时诊断】I2C 读健康度，由 bsp_modbus 发布到 0x0184~0x018C。定位完整段删除。 */
extern volatile uint16_t g_mag_i2c_ok;       /* 成功累计 */
extern volatile uint16_t g_mag_i2c_err;      /* 失败累计 */
extern volatile uint8_t  g_mag_i2c_hal;      /* 最后一次 HAL 返回码：0=OK 1=ERROR 2=BUSY 3=TIMEOUT */
extern volatile uint8_t  g_mag_i2c_state;    /* 读结束后 hi2c3.State */
extern volatile uint8_t  g_mag_i2c_errcode;  /* 读结束后 hi2c3.ErrorCode */
extern volatile uint16_t g_mag_i2c_recover;  /* 【自愈】I2C3 复位次数 */

/* 【自愈 2026-09-23】STM32F4 的 I2C 外设一旦被总线毛刺打到 BUSY 卡死，
 * HAL 只会一直返回 HAL_BUSY —— 【永不自愈】。本函数做三件事：
 *   ① 9 个时钟总线恢复（把 SCL 临时当 GPIO，逼从机放开 SDA）
 *   ② HAL_I2C_DeInit + MX_I2C3_Init 复位外设（清掉内部 BUSY 状态）
 *   ③ 计数 +1
 * `ist8310_IIC_read_muli_reg()` 在【连续失败 3 次】时自动调用。 */
extern void ist8310_i2c_recover(void);

#endif
