/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART6 init function —— RS485 Modbus RTU 从站（当前使用）
 *
 * ⚠️ 先说清 C 板丝印与 MCU 外设的【交叉错位】关系，官方手册原文：
 *   "开发板的外壳丝印（UART1 与 UART2）与 STM32 的实际串口配置并不对应，
 *    外壳丝印 UART1 对应 STM32 的 UART6，外壳丝印 UART2 对应 STM32 的 UART1。"
 *
 *   | 外壳丝印        | 实际 MCU 外设 | 引脚          | 脚序            | 供电 |
 *   |-----------------|--------------|---------------|-----------------|------|
 *   | UART1 (3-pin)   | **USART6**   | PG14 / PG9    | GND-TXD-RXD     | 无   |
 *   | UART2 (4-pin)   | USART1       | PA9 / PB7     | RXD-TXD-GND-5V  | 5V   |
 *
 * 本工程接的是**外壳丝印 UART1 的 3-pin 口** → MCU 侧是 USART6：
 *   PG14 = USART6_TX（AF8）→ 接 TTL-RS485 模块的 RXD/DI
 *   PG9  = USART6_RX（AF8）→ 接 TTL-RS485 模块的 TXD/RO
 *
 * 注意：USART6 的 TX/RX **不是** PA11/PA12（那两个是 USB_OTG_FS 的 D-/D+），
 * 以官方手册接口附表为准。
 *
 * RS485 方向控制仍用 PG8。但 3-pin 口本身只引出 GND/TXD/RXD 三根，
 * PG8 需从板内焊盘/排针另引；若 TTL-RS485 模块自带自动收发方向，PG8 可以不接。
 *
 * 另：3-pin 口**没有电源脚**，模块需另找 5V/3.3V 供电。 */

/* 按给定波特率/校验重新初始化 USART6（对齐 running 的 MX_USART2_UART_Init_With）。
 * 参数来自保持寄存器 0x0090 枚举查表（见 bsp_modbus.c 的 uart_cfg_table）。
 * 注意：本工程 RS485 收发是裸寄存器自管中断，
 * HAL_UART_Init 只负责配波特率/帧格式/GPIO/NVIC，不接管收发中断。 */
void MX_USART6_RS485_UART_Init_With(uint32_t baud, uint32_t parity)
{
  huart2.Instance = USART6;
  huart2.Init.BaudRate = baud;
  /* 奇偶校验模式下数据位自动降为 7 位（HAL 要求：WordLength 是"含校验位"的口径） */
  huart2.Init.WordLength = (parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = parity;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* 默认参数（115200 8N1）初始化，供上电时未读到有效配置枚举时兜底 */
void MX_USART6_RS485_UART_Init(void)
{
  MX_USART6_RS485_UART_Init_With(115200U, UART_PARITY_NONE);
}

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART6)
  {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX   (C 板外壳丝印 UART1 的 3-pin 口 TXD)
    PG9      ------> USART6_RX   (C 板外壳丝印 UART1 的 3-pin 口 RXD)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* RS485 方向控制脚（PG8，需从板内焊盘/排针另引）。
     * 外部 TTL-RS485 模块自带自动方向时此脚悬空无影响。 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);

    /* USART6 在 APB2 总线（与 USART1 同为 APB2；USART2/3 在 APB1） */
    HAL_NVIC_SetPriority(USART6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART6)
  {
    __HAL_RCC_USART6_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14);
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_8);
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
