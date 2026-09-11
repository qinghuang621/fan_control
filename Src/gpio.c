/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* BMI088 使用 PB3(SPI1_SCK) / PB4(SPI1_MISO)。
   * 这两个脚上电默认复用为 JTAG 的 JTDO/NJTRST，但在 STM32F4 上，
   * 只要在 HAL_SPI_MspInit 里把 AFR 配成 GPIO_AF5_SPI1，硬件即自动切换为 SPI 功能，
   * **不需要** F1 那套 AFIO_MAPR 的 SWJ_CFG 重映射（那是 F1 独有的寄存器）。
   * 注意：此处绝不能调用 JTAGDISABLE 类操作，否则会连 SWD 一起关掉，下载器将失联。 */

  /* USER CODE END MX_GPIO_Init_1 */

  /*Configure GPIO pin Output Level --------------------------------------------*/
  /* BMI088 两路片选默认拉高（空闲态），避免上电瞬间误选中从机 */
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,  CS1_GYRO_Pin,  GPIO_PIN_SET);

  /*Configure GPIO pins : CS1_ACCEL_Pin CS1_GYRO_Pin ---------------------------*/
  GPIO_InitStruct.Pin   = CS1_ACCEL_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS1_ACCEL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS1_GYRO_Pin;
  HAL_GPIO_Init(CS1_GYRO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_ACCEL_Pin INT1_GYRO_Pin -------------------------*/
  /* 陀螺/加速度计的数据就绪中断，下降沿有效，内部上拉 */
  GPIO_InitStruct.Pin  = INT1_ACCEL_Pin | INT1_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG0 ---------------------------------------------------
   * ⚠️⚠️ 这一行是采集链路的**命门**，删掉则姿态角恒为 0、温度恒为 0。
   *
   * PG0 在本工程不接任何外设，唯一用途是给 EXTI 提供一条"可软触发的线"：
   * SPI1 DMA 收完一帧后，DMA2_Stream2_IRQHandler 调
   *     __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0)
   * （宏展开就是 EXTI->SWIER |= GPIO_PIN_0）来唤醒阻塞在 ulTaskNotifyTake 的 InsTask。
   *
   * 关键点：SWIER 只是**把 PR0 挂起位置 1**，真正送达 NVIC 还要过 IMR（中断屏蔽寄存器）。
   * 而 IMR0 只有在该线被配成 EXTI 输入模式（GPIO_MODE_IT_xxx）时才会被 HAL 置位。
   * 所以**必须**保留这行把 PG0 配成 IT 模式——哪怕它没接东西。
   * 只调 HAL_NVIC_EnableIRQ(EXTI0_IRQn) 是不够的：NVIC 开了但 IMR0=0，
   * 软触发永远到不了中断服务函数 → 现象就是"通信全通、状态字=2(RUNNING)，
   * 但 roll/pitch/温度恒 0、加热 PWM 卡死在 4499"。
   * 参见 DJI 例程 18.ins_task/Src/gpio.c：那里把 GPIO_PIN_0 与 DRDY_IST8310 一起
   * 配成了 GPIO_MODE_IT_FALLING（同在 GPIOG），本工程照此对齐。 */
  GPIO_InitStruct.Pin  = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init -------------------------------------------------------*/
  /* ⚠️ 这两个中断是**采集链路的唯一触发器**，必须使能！
   * BMI088 每完成一次转换就拉低对应 INT1 引脚，中断里才发起 SPI1 DMA。
   * 若这里不使能，DMA 永远不会被启动，姿态角将恒为 0。 */
  HAL_NVIC_SetPriority(INT1_ACCEL_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(INT1_ACCEL_EXTI_IRQn);

  HAL_NVIC_SetPriority(INT1_GYRO_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(INT1_GYRO_EXTI_IRQn);

  /* EXTI0 软中断：由 SPI1 DMA 接收完成中断里 __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0)
   * 软件触发，用于把 InsTask 从 ulTaskNotifyTake 中唤醒。
   * 优先级 6：数值比 DMA 的 5 大 = 优先级更低，保证 DMA 链路先跑完。
   * EXTI0 线由上面的 PG0 提供（未接外设，纯软件触发，无冲突）。 */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
