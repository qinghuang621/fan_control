#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "struct_typedef.h"

/**
 * @brief  初始化 SPI1 的收发 DMA 通道（配置外设地址、内存地址、数据长度）
 * @note   该函数从例程原样移植。它绕过 HAL 的 DMA 状态机直接裸写寄存器，
 *         因为 BMI088 的采样时序要求极低延迟，走 HAL_SPI_TransmitReceive_DMA
 *         的状态检查会引入不确定开销。
 * @param  tx_buf 发送缓冲区地址
 * @param  rx_buf 接收缓冲区地址
 * @param  num    本次传输字节数
 */
void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);

/**
 * @brief  重新装载缓冲区地址与长度并启动一次 DMA 收发
 * @note   每次采样前调用，可复用同一对缓冲区而不必重复初始化
 */
void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif
