#ifndef BSP_ADS1256_H
#define BSP_ADS1256_H

#include <stdint.h>

/* ADS1256 状态码（与 Modbus 0x01A8 的 ADS_STATUS 一致） */
#define ADS1256_OK          0U
#define ADS1256_ERR_DRDY    1U  /* 等 DRDY 超时 */
#define ADS1256_ERR_INIT    2U  /* 未初始化/初始化失败 */
#define ADS1256_ERR_SPI     3U  /* SPI 传输失败 */

/* 一次四路扫描的快照。电压单位 V，raw 为 24bit 有符号码值。
 * v5v / pressure 当前未实现，保持 0；预留与文档 0x01BA~0x01C3 对应。 */
typedef struct
{
    uint16_t status;      /* ADS1256_OK / ERR_* */
    uint16_t seq;         /* 每次成功扫描 +1 */
    int32_t  raw[4];      /* AIN0~AIN3 原始码 */
    float    volt[4];     /* AIN0~AIN3 电压（ACOM=0V 单端，不加 2.5） */
    float    v5v;         /* 5V 母线（未实现，0） */
    float    pressure[4]; /* 工程量（未实现，0） */
} ads1256_snapshot_t;

/* 初始化：配置 PB12(CS)/PF1(DRDY)，可选 PF0(RST)，然后软复位 + 寄存器 + 自校准。
 * 返回 ADS1256_OK 表示成功。调用前必须先 MX_SPI2_Init()。 */
uint8_t bsp_ads1256_init(void);

/* 是否已初始化成功 */
uint8_t bsp_ads1256_is_ready(void);

/* 读单通道（0~3）。内部会切 MUX、SYNC/WAKEUP，并丢弃切换后的第一拍。 */
uint8_t bsp_ads1256_read_ch(uint8_t ch, int32_t *code);

/* 扫描 AIN0~AIN3，填充快照。返回 ADS1256_OK 表示四路都成功。 */
uint8_t bsp_ads1256_scan(ads1256_snapshot_t *out);

/* 24bit 码值转电压：V = code * 5.0 / 8388608（PGA=1，VREF=2.5V，单端） */
float bsp_ads1256_code_to_volt(int32_t code);

#endif /* BSP_ADS1256_H */
