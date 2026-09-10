#include "BMI088Middleware.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

/* 【相对例程的改动】
 *   1. 去掉 cmsis_os.h / bsp_delay.h 依赖。C 板未编 cmsis_os2.c（裸 FreeRTOS），
 *      也没有例程那套基于 SysTick 的 bsp_delay（C 板 SysTick 已交给 FreeRTOS 调度器）。
 *      此处：
 *        - BMI088_delay_ms  -> vTaskDelay（毫秒级，用于初始化期间的寄存器稳定等待）
 *        - BMI088_delay_us  -> DWT 周期计数器忙等（微秒级，不能进调度器，否则会
 *                              在初始化阶段时序失控）
 *   2. 保留 extern SPI_HandleTypeDef hspi1（C 板由 Src/spi.c 提供）。 */

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{
    /* 片选与 EXTI 已在 MX_GPIO_Init 中统一配置，此处留空 */
}

void BMI088_com_init(void)
{
    /* SPI1 已在 MX_SPI1_Init 中初始化，此处留空 */
}

void BMI088_delay_ms(uint16_t ms)
{
    /* 注意：该函数在任务上下文调用（BMI088_init 在 InsTask 中执行）。
     * 若在调度器启动前调用需改用忙等 —— 当前所有调用点都在任务里，安全。 */
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

/* DWT CYCCNT 微秒忙等。
 * 用 DWT 而不是 TIM：TIM 资源已排满（TIM1/5/6/8/10），且 DWT 不需要占用中断。
 * DWT 初始化见 bsp_dwt_init()。 */
#define DWT_CTRL        (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT      (*(volatile uint32_t *)0xE0001004)
#define CoreDebug_DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA    (1UL << 24)
#define DWT_CTRL_CYCCNTENA (1UL << 0)

void bsp_dwt_init(void)
{
    CoreDebug_DEMCR |= DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t start = DWT_CYCCNT;
    /* 168MHz -> 1us = 168 周期 */
    uint32_t ticks = (uint32_t)us * (SystemCoreClock / 1000000U);

    /* CYCCNT 为 32 位，约 25.5s 回绕一次；此处等待最长 ~65ms，无需处理回绕 */
    while ((DWT_CYCCNT - start) < ticks)
    {
        __NOP();
    }
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
