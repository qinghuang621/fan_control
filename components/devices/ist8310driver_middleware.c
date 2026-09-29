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

/* 【临时诊断 2026-09-23】原版是 void 且吞掉了 HAL 返回值 —— 读失败时 buf 保持
 * 原样，调用方却照常解析，于是输出"看起来像巨大磁场的垃圾"，且 MAG_STATUS 仍报
 * 九轴在跑，故障完全静默。现返回状态并把 I2C 现场记录下来。 */
volatile uint16_t g_mag_i2c_ok      = 0U;
volatile uint16_t g_mag_i2c_err     = 0U;
volatile uint8_t  g_mag_i2c_hal     = 0U;   /* HAL_StatusTypeDef */
volatile uint8_t  g_mag_i2c_state   = 0U;   /* hi2c3.State */
volatile uint8_t  g_mag_i2c_errcode = 0U;   /* hi2c3.ErrorCode */
volatile uint16_t g_mag_i2c_recover = 0U;   /* 【自愈】I2C3 复位次数 */

/* 【自愈 2026-09-23】连续失败达到此值就复位 I2C3。
 * 取 3 的理由：单次孤立失败（毛刺但没卡死）不会触发（下一次就成功了），
 * 而"卡死"表现为每次调用都失败 ⇒ 3 次即可判定，约 75ms 内完成自愈。 */
#define MAG_I2C_FAIL_LIMIT  3U
static uint8_t s_mag_i2c_fail_run = 0U;     /* 连续失败计数（成功即清零） */

/* 9 个时钟的总线恢复。
 * 为什么不能只复位主机外设：I2C 从机若在传输中途被复位/干扰，可能**死拉住 SDA 不放**，
 * 此时总线仍被从机占着，光复位 STM32 的 I2C 外设没用 —— 必须先给它补够时钟，
 * 让它把剩下的位移完、自己放开 SDA。做法：把 SCL/SDA 临时切成普通开漏输出。 */
static void mag_i2c_bus_recover(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = GPIO_PIN_8;          /* PA8 = I2C3_SCL */
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_9;          /* PC9 = I2C3_SDA */
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   /* 放开 SDA */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   /* SCL 高 */
    ist8310_delay_us(10U);

    for (i = 0U; i < 9U; ++i)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        ist8310_delay_us(10U);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        ist8310_delay_us(10U);

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET)
        {
            break;                  /* 从机已放开 SDA，收手 */
        }
    }

    /* 补一个 STOP：SCL 为高时把 SDA 由低拉高 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    ist8310_delay_us(10U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    ist8310_delay_us(10U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    ist8310_delay_us(10U);
}

void ist8310_i2c_recover(void)
{
    HAL_I2C_DeInit(&hi2c3);         /* ① 复位外设：清掉内部 BUSY 卡死状态 */
    mag_i2c_bus_recover();          /* ② 再把从机从总线上"敲"下来 */
    MX_I2C3_Init();                 /* ③ 重新初始化（MspInit 会把 PA8/PC9 配回 AF4）*/

    ist8310_delay_ms(2U);           /* 从机刚被折腾过，给点内部稳定时间 */

    g_mag_i2c_recover++;
    s_mag_i2c_fail_run = 0U;
}

uint8_t ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, len, 100);

    g_mag_i2c_hal     = (uint8_t)st;
    g_mag_i2c_state   = (uint8_t)hi2c3.State;
    g_mag_i2c_errcode = (uint8_t)hi2c3.ErrorCode;

    if (st != HAL_OK)
    {
        g_mag_i2c_err++;

        /* 【自愈】STM32F4 的 I2C 一旦被毛刺打到 BUSY 卡死，HAL 只会永远回 HAL_BUSY，
         * 不主动复位就是【永久失效】—— 实测 MAG_OK 会一直冻结在卡死那一刻。 */
        if (++s_mag_i2c_fail_run >= MAG_I2C_FAIL_LIMIT)
        {
            ist8310_i2c_recover();
        }
        return 0U;
    }

    s_mag_i2c_fail_run = 0U;
    g_mag_i2c_ok++;
    return 1U;
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
