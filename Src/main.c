/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fric.h"
#include "bsp_led.h"
#include "bsp_led_tilt.h"
#include "bsp_modbus.h"
#include "bsp_pulse.h"
#include "bsp_ads1256.h"
#include "ins_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
static void ModbusTask_Entry(void *argument);
static void PulseTask_Entry(void *argument);
static void LedTask_Entry(void *argument);
#if ADS1256_ENABLE
static void AdcTask_Entry(void *argument);
#endif
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t idleTaskTCB;
    static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &idleTaskTCB;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    Error_Handler();
}

/* TIM6 每 1ms 触发一次更新中断，这里给 HAL 的 uwTick 计数（HAL_Delay/超时用）。
 * SysTick 已交给 FreeRTOS，不再承担 HAL 时基。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

/* USER CODE END 0 */

static void ModbusTask_Entry(void *argument)
{
   (void)argument;
   for (;;)
   {
       modbus_poll();
       vTaskDelay(pdMS_TO_TICKS(5U));
   }
}

static void PulseTask_Entry(void *argument)
{
   (void)argument;
   for (;;)
   {
       pulse_poll();
       vTaskDelay(pdMS_TO_TICKS(10U));
   }
}

static void LedTask_Entry(void *argument)
{
   (void)argument;
   for (;;)
   {
       /* 倾斜指示灯：色相=倾斜方向，饱和度=幅度，亮度=系统状态
        * （内部限流到 50Hz，5ms 调一次只是让限流更平滑）
        * 注意：彩虹呼吸灯 led_tick() 已移除，两者争用同一组 TIM5 通道 */
       led_tilt_update();
       vTaskDelay(pdMS_TO_TICKS(5U));
   }
}

/* ADS1256 四路扫描任务。
 * 优先级 +1（最低，与 LedTask 同级）；低于 ModbusTask(+4)/InsTask(+3)/PulseTask(+2)。
 * 一轮扫描约 12~16ms（1kSPS、每路丢弃第一拍并等 1ms），约 60~80Hz/通道。 */
#if ADS1256_ENABLE
static void AdcTask_Entry(void *argument)
{
   (void)argument;

   /* 保留最后一次成功扫描的数据：故障时只更新 status，不清零 SEQ/电压，
    * 这样上位机能区分“压力真的是 0”和“采集卡掉线”。 */
   static ads1256_snapshot_t s_last;
   s_last.status = ADS1256_ERR_INIT;

   for (;;)
   {
       ads1256_snapshot_t snap = {0};

       if (bsp_ads1256_is_ready() == 0U)
       {
           uint8_t ret = bsp_ads1256_init();
           if (ret != ADS1256_OK)
           {
               s_last.status = ret;          /* 1=DRDY 超时 2=初始化失败 3=SPI 错误 */
               modbus_ads_update(&s_last);
               vTaskDelay(pdMS_TO_TICKS(1000U));
               continue;
           }
           /* 初始化成功：直接往下扫描，不额外延时 */
       }

       if (bsp_ads1256_scan(&snap) != ADS1256_OK)
       {
           /* 扫描失败：保留上次成功的 SEQ/电压，只把错误码写进 status */
           s_last.status = snap.status;
           modbus_ads_update(&s_last);
           vTaskDelay(pdMS_TO_TICKS(100U));
           (void)bsp_ads1256_init();
           vTaskDelay(pdMS_TO_TICKS(1000U));
           continue;
       }

       s_last = snap;
       modbus_ads_update(&s_last);
       vTaskDelay(pdMS_TO_TICKS(1U));
   }
}
#endif /* ADS1256_ENABLE */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_RS485_UART_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
#if ADS1256_ENABLE
  MX_SPI2_Init();
#endif
  MX_TIM10_Init();
  MX_I2C3_Init();
  /* USB CDC 已废弃：CAN1 占用 PD0/PD1，且不再使用虚拟串口调试 */
  /* USER CODE BEGIN 2 */

    /* 启动 TIM8 CH1~CH3 PWM 输出（C板 PWM5~PWM7，20kHz） */
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

    /* 上电默认安全状态：PWM=0 */
    fric_off();

    /* 启动 PWM1~PWM4(PE9/PE11/PE13/PE14) 四路 FG 脉冲输入捕获 */
    pulse_capture_init();

    /* 初始化板载 RGB LED（TIM5 + PH10/PH11/PH12）。
     * 显示内容由 LedTask 里的 led_tilt_update() 决定（倾斜指示），
     * 彩虹呼吸灯 led_tick() 已移除 —— 二者争用同一组 TIM5 通道。 */
    led_init();

    /* 初始化 RS485 Modbus RTU 从站 */
    modbus_init();

    /* ADS1256 采集卡：SPI2(PB13/14/15) + CS(PB12) + DRDY(PF1)。
     *
     * 🔴 这里**不再**调用 bsp_ads1256_init()。
     * 原因：本函数位于 vTaskStartScheduler() **之前**，而 ADS 初始化里有多处
     * 阻塞等待（等 5V 稳定 100ms、软复位 100ms、DRDY 200/300ms、自校准 20ms）。
     * 万一其中任何一处变成死等（时基不前进、DRDY 一直不低），芯片就永远停在
     * 调度器启动之前 —— 所有任务都不跑，RS485 一个字节都不回，且从上位机看
     * 和"固件刷坏了"完全一样，极难定位。
     * 现在改成：全部初始化交给 AdcTask 首轮执行（任务里阻塞只影响它自己），
     * 失败就按 1s 周期重试，并把错误码写进 0x01A8，Modbus/风机/电机不受影响。 */

    /* BMI088 恒温加热 PWM（TIM10 CH1 / PF6）先启动，占空比由 InsTask 的 PID 接管 */
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    /* 任务优先级说明（tskIDLE_PRIORITY = 0）：
     *   ModbusTask +4  最高 —— RS485 通信超时敏感，且 CAN 发送跑在这里
     *   InsTask    +3     姿态解算
     *   PulseTask  +2     FG 脉冲统计
     *   LedTask    +1     最低 —— 纯装饰，被抢占无影响
     *
     * 原 FanTask（+3，风机斜坡软启动）已删除：其入口 fan_set_target() 从未被调用，
     * 状态机恒为 IDLE、每 5ms 空转一次。风机输出实际由 fan_auto_update() /
     * sync_fan_outputs_from_regs() 在 ModbusTask 里直接写 CCR。 */
    if (xTaskCreate(ModbusTask_Entry, "ModbusTask", 256U, NULL, tskIDLE_PRIORITY + 4U, NULL) != pdPASS)
    {
        Error_Handler();
    }
    if (xTaskCreate(PulseTask_Entry, "PulseTask", 256U, NULL, tskIDLE_PRIORITY + 2U, NULL) != pdPASS)
    {
        Error_Handler();
    }
    /* ADS1256 四路扫描：**最低优先级(+1)**，与 LedTask 同级。
     * 采集是纯诊断/慢变量（每通道 60~80Hz 已经远快于压力变化），
     * 没有任何理由让它和 Modbus(+4)/Ins(+3)/Pulse(+2) 抢 CPU；
     * 降到 +1 后即使它偶发长时间阻塞，也抢不走通信任务的时间片。 */
#if ADS1256_ENABLE
    if (xTaskCreate(AdcTask_Entry, "AdcTask", 384U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
    {
        Error_Handler();
    }
#endif
    if (xTaskCreate(LedTask_Entry, "LedTask", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
    {
        Error_Handler();
    }
    /* InsTask 栈需放大：内部有 mahony（含 sqrt/atan2 浮点库调用）、PID、
     * 以及 seqlock 快照临时结构，256 字（1KB）偏紧。 */
    if (xTaskCreate(InsTask_Entry, "InsTask", 512U, NULL, tskIDLE_PRIORITY + 3U, NULL) != pdPASS)
    {
        Error_Handler();
    }

    vTaskStartScheduler();

    /* Should never reach here. */
    for (;;)
    {
    }

  /* USER CODE END 2 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 红光无限闪烁（不返回），替代原来的空实现。
   * 原来外设初始化失败会静默停在空函数里，现象是"板子像没跑起来"，
   * 容易误判为硬件故障；现在一眼能看出是软件侧初始化失败。
   * 注意：led_fatal_blink() 自己配置 GPIO，不依赖 led_init()。 */
  led_fatal_blink();

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
