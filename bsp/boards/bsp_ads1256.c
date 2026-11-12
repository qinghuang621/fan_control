#include "bsp_ads1256.h"
#include "spi.h"

/* 黄色 ADS1256 采集卡接线（与文档一致）：
 *   C 板 PB12 -> 黄板 CS   (第 7 脚)
 *   C 板 PB13 -> 黄板 SCLK (第 3 脚)
 *   C 板 PB15 -> 黄板 DIN  (第 4 脚)
 *   C 板 PB14 -> 黄板 DOUT (第 5 脚)
 *   C 板 PF1  -> 黄板 DROY (第 6 脚，DRDY)
 *   C 板 PF0  -> 黄板 RST  (第 8 脚，实测 3.3V 上拉，默认不接)
 *
 * 注意：5V/GND 不在 C 板 8-pin 口内，必须另外从 4-pin UART2/CAN2 取。 */

#define ADS_CS_PORT       GPIOB
#define ADS_CS_PIN        GPIO_PIN_12
#define ADS_DRDY_PORT     GPIOF
#define ADS_DRDY_PIN      GPIO_PIN_1
#define ADS_RST_PORT      GPIOF
#define ADS_RST_PIN       GPIO_PIN_0

/* RST 实测 3.3V 上拉，默认不接 PF0；只有需要硬复位时才改成 1 */
#ifndef ADS1256_USE_HW_RESET
#define ADS1256_USE_HW_RESET 0
#endif

#define ADS_CS_LOW()      HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_RESET)
#define ADS_CS_HIGH()     HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_SET)
#define ADS_DRDY_READ()   HAL_GPIO_ReadPin(ADS_DRDY_PORT, ADS_DRDY_PIN)

/* ADS1256 命令字 */
#define ADS_CMD_RDATA     0x01U
#define ADS_CMD_SDATAC    0x0FU
#define ADS_CMD_WREG      0x50U
#define ADS_CMD_SELFCAL   0xF0U
#define ADS_CMD_SYNC      0xFCU
#define ADS_CMD_RESET     0xFEU
#define ADS_CMD_WAKEUP    0x00U   /* ADS1256 WAKEUP=0x00；0xFF 不是有效命令 */

/* 寄存器地址（ADS1256：STATUS=0x00, MUX=0x01, ADCON=0x02, DRATE=0x03） */
#define ADS_REG_STATUS    0x00U
#define ADS_REG_MUX       0x01U
#define ADS_REG_ADCON     0x02U
#define ADS_REG_DRATE     0x03U
#define ADS_MUX_AINCOM    0x08U

/* DRATE = 1kSPS：四路 + 每路丢弃第一拍，扫描约 8ms */
#define ADS_DRATE_1KSPS   0xA1U

static uint8_t  s_ready = 0U;
static uint16_t s_seq = 0U;

static uint8_t ads_spi_tx(const uint8_t *buf, uint16_t len)
{
    return (HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, len, 10U) == HAL_OK) ? 0U : 1U;
}

static uint8_t ads_spi_rx(uint8_t *buf, uint16_t len)
{
    return (HAL_SPI_Receive(&hspi2, buf, len, 10U) == HAL_OK) ? 0U : 1U;
}

static uint8_t ads_write_cmd(uint8_t cmd)
{
    uint8_t ret;

    ADS_CS_LOW();
    ret = ads_spi_tx(&cmd, 1U);
    ADS_CS_HIGH();
    return ret;
}

static uint8_t ads_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t buf[3];
    uint8_t ret;

    buf[0] = (uint8_t)(ADS_CMD_WREG | addr);
    buf[1] = 0x00U;   /* count-1 = 0，写 1 个寄存器 */
    buf[2] = value;

    ADS_CS_LOW();
    ret = ads_spi_tx(buf, 3U);
    ADS_CS_HIGH();
    return ret;
}

/* 约 us 微秒的忙等（168MHz，42 次循环约 1µs；偏保守） */
static void ads_delay_us(uint32_t us)
{
    volatile uint32_t n = us * 42U;
    while (n-- != 0U)
    {
        __NOP();
    }
}

static uint8_t ads_read_data(int32_t *code)
{
    uint8_t cmd = ADS_CMD_RDATA;
    uint8_t rx[3] = {0U, 0U, 0U};
    int32_t v;

    ADS_CS_LOW();
    if (ads_spi_tx(&cmd, 1U) != 0U)
    {
        ADS_CS_HIGH();
        return 1U;
    }
    /* RDATA 后给 t6 余量（参考代码要求约 50/fCLKIN ≈ 6.5µs），再读 3 字节 */
    ads_delay_us(10U);
    if (ads_spi_rx(rx, 3U) != 0U)
    {
        ADS_CS_HIGH();
        return 1U;
    }
    ADS_CS_HIGH();

    v = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | (int32_t)rx[2];
    if ((v & 0x00800000L) != 0L)
    {
        v |= (int32_t)0xFF000000L;   /* 24bit 符号扩展 */
    }
    *code = v;
    return 0U;
}

static uint8_t ads_wait_drdy(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (ADS_DRDY_READ() != GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - t0) >= timeout_ms)
        {
            return 1U;
        }
        HAL_Delay(1U);
    }
    return 0U;
}

static void ads_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* CS 默认拉高，避免上电瞬间误选中 */
    ADS_CS_HIGH();
    gpio.Pin   = ADS_CS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ADS_CS_PORT, &gpio);

    /* DRDY 输入上拉；黄板推挽输出，低=新数据就绪 */
    gpio.Pin  = ADS_DRDY_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ADS_DRDY_PORT, &gpio);

#if ADS1256_USE_HW_RESET
    /* RST 实测 3.3V 上拉，默认不接；只有接 PF0 时才配成输出 */
    HAL_GPIO_WritePin(ADS_RST_PORT, ADS_RST_PIN, GPIO_PIN_SET);
    gpio.Pin   = ADS_RST_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ADS_RST_PORT, &gpio);
#endif
}

uint8_t bsp_ads1256_init(void)
{
    int32_t dummy;

    s_ready = 0U;
    ads_gpio_init();

    HAL_Delay(100U);   /* 等采集卡 5V 和内部基准稳定 */

#if ADS1256_USE_HW_RESET
    HAL_GPIO_WritePin(ADS_RST_PORT, ADS_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(2U);
    HAL_GPIO_WritePin(ADS_RST_PORT, ADS_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(100U);
#else
    if (ads_write_cmd(ADS_CMD_RESET) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    HAL_Delay(100U);
#endif

    if (ads_wait_drdy(200U) != 0U)
    {
        return ADS1256_ERR_DRDY;
    }

    /* 退出连续读；STATUS=0x04(ORDER=0/ACAL=1/BUFEN=0)、ADCON=0x00(PGA=1)、DRATE=0xA1(1kSPS)
     *
     * ⚠ BUFEN 必须为 0：AVDD=5V 时，缓冲开的绝对输入电压上限是 AVDD-2V = 3.0V
     * （TI 手册：between AGND and AVDD-2.0V）；传感器 0.5~4.5V 会超限，线性度下降。
     * BUFEN=0 时 0~5V 量程才成立（商家标的 0~5V 就是这个条件）。 */
    if (ads_write_cmd(ADS_CMD_SDATAC) != 0U)
    {
        return ADS1256_ERR_SPI;
    }

    /* 必须分开写：ADS1256 的 MUX 在 0x01，夹在 STATUS(0x00) 与 ADCON(0x02) 之间。
     * 从 0x00 连写 3 个会把 MUX 写成 0、把 DRATE 写进 ADCON。 */
    if (ads_write_reg(ADS_REG_STATUS, 0x04U) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    if (ads_write_reg(ADS_REG_ADCON, 0x00U) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    if (ads_write_reg(ADS_REG_DRATE, ADS_DRATE_1KSPS) != 0U)
    {
        return ADS1256_ERR_SPI;
    }

    if (ads_write_cmd(ADS_CMD_SELFCAL) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    /* 等自校准先拉高 DRDY 再拉低；先给一段固定延时，避免读到校准前的旧低电平 */
    HAL_Delay(20U);
    if (ads_wait_drdy(300U) != 0U)
    {
        return ADS1256_ERR_DRDY;
    }

    /* 先选 AIN0/ACOM，并读掉切换后的第一拍 */
    if (ads_write_reg(ADS_REG_MUX, (uint8_t)((0U << 4) | ADS_MUX_AINCOM)) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    /* SYNC 与 WAKEUP 分两条命令发（与参考代码一致，CS 每条命令都拉高一次） */
    if (ads_write_cmd(ADS_CMD_SYNC) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    if (ads_write_cmd(ADS_CMD_WAKEUP) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    /* SYNC 后 DRDY 先拉高；给 1ms 让新的转换完成，避免读到旧的 DRDY 低电平 */
    HAL_Delay(1U);
    if (ads_wait_drdy(20U) != 0U)
    {
        return ADS1256_ERR_DRDY;
    }
    if (ads_read_data(&dummy) != 0U)
    {
        return ADS1256_ERR_SPI;
    }

    s_ready = 1U;
    return ADS1256_OK;
}

uint8_t bsp_ads1256_is_ready(void)
{
    return s_ready;
}

uint8_t bsp_ads1256_read_ch(uint8_t ch, int32_t *code)
{
    int32_t dummy;

    if ((s_ready == 0U) || (ch > 3U) || (code == 0))
    {
        return ADS1256_ERR_INIT;
    }

    /* 正端 AINch，负端 ACOM（AINCOM=8） */
    if (ads_write_reg(ADS_REG_MUX, (uint8_t)((ch << 4) | ADS_MUX_AINCOM)) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    /* SYNC 与 WAKEUP 分两条命令发（与参考代码一致，CS 每条命令都拉高一次） */
    if (ads_write_cmd(ADS_CMD_SYNC) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    if (ads_write_cmd(ADS_CMD_WAKEUP) != 0U)
    {
        return ADS1256_ERR_SPI;
    }
    /* SYNC 后 DRDY 先拉高；给 1ms 让新的转换完成，避免读到旧的 DRDY 低电平 */
    HAL_Delay(1U);

    /* 切换 MUX 后第一拍丢弃 */
    if (ads_wait_drdy(20U) != 0U)
    {
        return ADS1256_ERR_DRDY;
    }
    if (ads_read_data(&dummy) != 0U)
    {
        return ADS1256_ERR_SPI;
    }

    /* 第二拍才是有效数据 */
    if (ads_wait_drdy(20U) != 0U)
    {
        return ADS1256_ERR_DRDY;
    }
    if (ads_read_data(code) != 0U)
    {
        return ADS1256_ERR_SPI;
    }

    return ADS1256_OK;
}

uint8_t bsp_ads1256_scan(ads1256_snapshot_t *out)
{
    if (out == 0)
    {
        return ADS1256_ERR_INIT;
    }

    out->status = ADS1256_ERR_INIT;
    out->v5v = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        out->raw[i] = 0;
        out->volt[i] = 0.0f;
        out->pressure[i] = 0.0f;
    }

    if (s_ready == 0U)
    {
        return ADS1256_ERR_INIT;
    }

    for (uint8_t ch = 0U; ch < 4U; ++ch)
    {
        int32_t code = 0;
        uint8_t ret = bsp_ads1256_read_ch(ch, &code);
        if (ret != ADS1256_OK)
        {
            out->status = ret;
            return ret;
        }
        out->raw[ch] = code;
        out->volt[ch] = bsp_ads1256_code_to_volt(code);
    }

    out->status = ADS1256_OK;
    out->seq = ++s_seq;
    return ADS1256_OK;
}

float bsp_ads1256_code_to_volt(int32_t code)
{
    /* ACOM=0V 单端；PGA=1、VREF=2.5V -> 满量程 5V */
    return ((float)code) * 5.0f / 8388608.0f;
}
