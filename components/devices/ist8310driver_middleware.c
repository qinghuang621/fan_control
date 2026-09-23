/**
 * @file       IST8310Middleware.c
 * @brief      IST8310 磁力计通信中间层，硬件 I2C3。
 *             风格对齐 BMI088Middleware.c：
 *               - 延时复用 FreeRTOS vTaskDelay + DWT CYCCNT 忙等
 *               - GPIO/I2C init 在 CubeMX MX_* 中完成，此处留空
 */

#include "ist8310driver_middleware.h"
#include "main.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

extern I2C_HandleTypeDef hi2c3;

void ist8310_GPIO_init(void)
{
    /* PG3/PG6 已在 MX_GPIO_Init 中配置，此处留空 */
}

void ist8310_com_init(void)
{
    /* I2C3 已在 MX_I2C3_Init 中初始化，此处留空 */
}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}

void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void ist8310_delay_ms(uint16_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        volatile uint32_t i;
        for (i = 0; i < (uint32_t)ms * 20000U; i++) { __NOP(); }
    }
}

/* DWT CYCCNT 微秒忙等 —— 复用 BMI088Middleware 里同名函数；
 * 若发现冲突可独立实现（DWT 寄存器地址相同）。 */
extern void BMI088_delay_us(uint16_t us);

void ist8310_delay_us(uint16_t us)
{
    BMI088_delay_us(us);
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}

void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}
