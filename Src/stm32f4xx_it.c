/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
/* HAL 时基由 TIM6 提供（见 Src/stm32f4xx_hal_timebase_tim.c），SysTick 交给 FreeRTOS */
extern TIM_HandleTypeDef htim6;
/* FreeRTOS 节拍入口，由 port.c 提供 */
extern void xPortSysTickHandler(void);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* 清除 SysTick 溢出标志（读 CTRL 即可），然后交给 FreeRTOS 节拍处理。
   * 调度器未启动时不能调用 xPortSysTickHandler，否则会在启动阶段触发上下文切换。 */
  SysTick->CTRL;

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
      xPortSysTickHandler();
  }

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/* USB CDC 已废弃（CAN 占用 PD0/PD1，不再使用虚拟串口），OTG_FS_IRQHandler 已移除 */

/* USER CODE BEGIN 1 */

#include "tim.h"
#include "spi.h"
#include "bsp_modbus.h"   /* usart6_irq_handler() —— RS485 裸寄存器中断服务 */

/* TIM1 捕获/比较中断（PWM1~PWM4 脉冲输入）
 * 注：只使用 CC 中断，不使用 TIM1 更新中断（TIM1_UP_TIM10） */
void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

/* USART6 全局中断：RS485 Modbus RTU 从站收发
 * （huart2 句柄即 USART6，见 Src/usart.c。
 *   C 板外壳丝印 UART1 的 3-pin 口 → MCU 实际是 USART6，PG14/PG9）
 *
 * 注意：这里**不调用 HAL_UART_IRQHandler**，而是走自管的裸寄存器中断，
 * 与 running 的 Middlewares/Third_Party/FreeModbus/modbus/port/portserial.c 一致：
 *   - 直接在中断里读 SR/DR，避免 HAL 状态机与裸寄存器操作冲突；
 *   - 每次中断都顺手清 ORE/FE/NE/PE，杜绝"一次溢出后接收永久停摆"；
 *   - TXE 逐字节发送、TC 才切回 RS485 接收方向，防止截断最后一位。 */
void USART6_IRQHandler(void)
{
    usart6_irq_handler();
}

/* CAN1 RX0 中断：接收达妙电机状态上报帧。
 * 走 HAL（HAL_CAN_IRQHandler → HAL_CAN_RxFifo0MsgPendingCallback），
 * 与 running 一致：中断里只把帧搬进 FreeRTOS 队列，寄存器更新在任务上下文做。 */
void CAN1_RX0_IRQHandler(void)
{
    motor_can_irq_handler();
}

/* TIM6 更新中断：HAL 1ms 时基（替代 SysTick） */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/* ---------------- BMI088 采集相关中断 ----------------
 * 注意：DMA2_Stream2/3 与 EXTI0/4/9_5 的处理函数**不在这里**，
 * 而是集中放在 components/algorithm/ins_task.c 中。
 * 原因：这些中断要直接读写采集状态机的位（gyro/accel/accel_temp_update_flag）
 * 并调用 imu_cmd_spi_dma()，与 INS 任务耦合很紧；
 * 且 SPI 收发是裸寄存器启动的（不经过 HAL 状态机），
 * 不能走 HAL_DMA_IRQHandler，否则 HAL 会认为"无传输"而直接返回，
 * 导致采集链路彻底停摆。集中在 ins_task.c 更清晰、也避免重复定义。 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
