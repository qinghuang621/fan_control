/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* ---------------- BMI088 (六轴 IMU) SPI1 片选 / 中断 ---------------- */
#define CS1_ACCEL_Pin              GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port        GPIOA

#define CS1_GYRO_Pin               GPIO_PIN_0
#define CS1_GYRO_GPIO_Port         GPIOB

#define INT1_ACCEL_Pin             GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port       GPIOC
#define INT1_ACCEL_EXTI_IRQn       EXTI4_IRQn

#define INT1_GYRO_Pin              GPIO_PIN_5
#define INT1_GYRO_GPIO_Port        GPIOC
#define INT1_GYRO_EXTI_IRQn        EXTI9_5_IRQn

/* IST8310 占位：本轮不接磁力计，但 PG 口引脚已在 C 板上被 TIM1 FG / PG8 RS485 占用，
 * 故此处不定义 RSTN/DRDY。下一轮接磁力计时再评估可用引脚。 */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
