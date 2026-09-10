#include "bsp_spi.h"
#include "main.h"
#include "spi.h"

/**
 * @brief  配置 SPI1 的收发 DMA 通道
 * @note   直接从例程移植。此处对 DMA 寄存器的操作绕过了 HAL 句柄状态机，
 *         __HAL_LINKDMA 建立的关联仅用于 SPI HAL 层识别 DMA 归属。
 *         注意：这里用 DMA_LISR_TCIF2 / DMA_LISR_TCIF3 常量清标志，
 *         对应 DMA2_Stream2(低寄存器 LISR 的 bit21-26) 与 DMA2_Stream3(bit27-31)。
 */
void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);

    __HAL_SPI_ENABLE(&hspi1);

    /* ---- 接收通道 DMA2_Stream2 ---- */
    __HAL_DMA_DISABLE(&hdma_spi1_rx);

    while (hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_LISR_TCIF2);

    hdma_spi1_rx.Instance->PAR  = (uint32_t) &(SPI1->DR);
    hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);

    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);

    /* ---- 发送通道 DMA2_Stream3 ---- */
    __HAL_DMA_DISABLE(&hdma_spi1_tx);

    while (hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);

    hdma_spi1_tx.Instance->PAR  = (uint32_t) &(SPI1->DR);
    hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);
}

/**
 * @brief  重新装载地址/长度并启动一次 DMA 收发
 * @note   每次采样前调用。长度由 ndtr 决定，因此同一对缓冲区可以承载
 *         陀螺(8字节) / 加速度计(9字节) / 温度(4字节) 三种不同长度的帧。
 */
void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    __HAL_DMA_DISABLE(&hdma_spi1_tx);

    while (hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }
    while (hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }

    /* 清各类标志，避免上一轮的残留标志误触发完成中断 */
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));

    __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));

    hdma_spi1_rx.Instance->M0AR = rx_buf;
    hdma_spi1_tx.Instance->M0AR = tx_buf;

    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);

    __HAL_DMA_ENABLE(&hdma_spi1_rx);
    __HAL_DMA_ENABLE(&hdma_spi1_tx);
}
