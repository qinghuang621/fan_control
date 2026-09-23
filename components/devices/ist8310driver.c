/**
 * @file    ist8310driver.c
 * @brief   IST8310 磁力计驱动，DJI 18.ins_task 例程移植。
 *          直接 I2C 读写（不经过 BMI088 的 I2C_SLVx 间接通道）。
 */

#include "ist8310driver.h"
#include "ist8310driver_middleware.h"

/* 灵敏度：原始计数 × 0.3 = μT */
#define MAG_SEN 0.3f

#define IST8310_WHO_AM_I       0x00
#define IST8310_WHO_AM_I_VALUE 0x10
#define IST8310_WRITE_REG_NUM  4

/* 配置寄存器写表（DJI 例程同款，逐寄存器写后回读校验） */
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] =
{
    {0x0B, 0x08, 0x01},
    {0x41, 0x09, 0x02},
    {0x42, 0xC0, 0x03},
    {0x0A, 0x0B, 0x04}
};

uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 1;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_GPIO_init();
    ist8310_com_init();

    /* 复位：RSTN 拉低 50ms 再拉高，等待 50ms 让内部稳定 */
    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();
    ist8310_delay_ms(sleepTime);

    /* WHO_AM_I 自检 */
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    ist8310_delay_ms(wait_time);

    /* 写配置寄存器并回读校验 */
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0],
                                     ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_ms(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_ms(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }

    return IST8310_NO_ERROR;
}

/* 从状态 buffer 解析磁力计（DJI 例程，通过 MPU6500 间接读的路径，本工程未使用） */
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{
    (void)status_buf;

    if (status_buf[0] & 0x01)
    {
        int16_t temp = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp;
        temp = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp;
        temp = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

/* 直接从 IST8310 读原始磁力计数据（本工程使用路径）
 *   寄存器 0x03 = X_L, 0x04 = X_H, ..., 0x08 = Z_H，共 6 字节
 *   16 位有符号 → 乘 0.3 → μT */
void ist8310_read_mag(fp32 mag[3])
{
    uint8_t buf[6];
    int16_t temp;

    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp;
    temp = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp;
    temp = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp;
}
