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

/* USART3（C 板 DBUS 接口，PC11）已删除：本工程遥控链路走 CRSF→ROS2→Modbus，
 * 不再使用 DBUS。原 CubeMX 配置为 100000 8N1，而 DBUS 实际要求 8E1，
 * 且 PC10(TX) 在 C 板上未连接。 */

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
 *   | DBUS (Pin-C8)   | USART3_RX    | PC11          | 与 PWM 共用     | 5V   |
 *
 * 注：DBUS 是 C 板第 3 个串口用途（手册接口编号 16），经反相电路接 USART3_RX，
 * 规定 100 kbps **8E1**（偶校验）。本工程不用 DBUS，此路已删除 —— 见上方说明。
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
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
