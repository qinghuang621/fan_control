/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "bsp_fric.h"
#include "bsp_pulse.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t s_rx_line[64];      /* 单行指令缓冲 */
static uint8_t s_rx_len = 0;       /* 当前行已收字节数 */
static uint8_t s_pulses_per_rev = 2; /* 风机每转脉冲数(FG)，默认 2，可用 PPRn 命令修改 */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static void cdc_process_command(const uint8_t *line, uint16_t len);
static void cdc_reply(const char *msg);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  uint32_t i;
  uint8_t byte;
  for (i = 0; i < *Len; i++)
  {
    byte = Buf[i];
    /* 行结束：支持 \n 或 \r\n */
    if (byte == '\n' || byte == '\r')
    {
      if (s_rx_len > 0)
      {
        s_rx_line[s_rx_len] = 0;
        cdc_process_command(s_rx_line, s_rx_len);
        s_rx_len = 0;
      }
    }
    else
    {
      if (s_rx_len < sizeof(s_rx_line) - 1)
      {
        s_rx_line[s_rx_len++] = byte;
      }
      else
      {
        /* 防止缓冲溢出 */
        s_rx_len = 0;
      }
    }
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* 回复消息到电脑端 */
static void cdc_reply(const char *msg)
{
  uint16_t len = (uint16_t)strlen(msg);
  if (len > 0)
  {
    CDC_Transmit_FS((uint8_t *)msg, len);
  }
}

/* 大小写不敏感比较 */
static int8_t str_ieq(const char *a, const uint8_t *b, uint16_t n)
{
  uint16_t i;
  for (i = 0; i < n; i++)
  {
    char ca = a[i];
    char cb = (char)b[i];
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return 0;
  }
  /* a 长度需等于 n */
  if (a[n] != 0) return 0;
  return 1;
}

/* 指令格式：
 *   START      -> 启动风机到默认 30% 启动占空比
 *   S0~S100    -> 设置目标占空比 0~100（启动后有效；S0 等于 STOP）
 *   STOP       -> 斜坡降到 0 并断电
 *   STATUS     -> 查询当前状态
 *   HELP       -> 指令列表
 */
static void cdc_process_command(const uint8_t *line, uint16_t len)
{
  /* START */
  if (str_ieq("START", line, len))
  {
    fan_set_target(FRIC_DUTY_START);
    cdc_reply("OK START\r\n");
    return;
  }
  /* STOP */
  if (str_ieq("STOP", line, len))
  {
    fan_stop();
    cdc_reply("OK STOP\r\n");
    return;
  }
  /* STATUS */
  if (str_ieq("STATUS", line, len))
  {
    static char buf[240];
    const char *st[] = {"IDLE", "RUN", "STOPPING"};
    int n;
    n = snprintf(buf, sizeof(buf),
                 "STATE=%s DUTY=%u TARGET=%u "
                 "FREQ1=%lu RPM1=%lu CNT1=%lu "
                 "FREQ2=%lu RPM2=%lu CNT2=%lu "
                 "FREQ3=%lu RPM3=%lu CNT3=%lu "
                 "FREQ4=%lu RPM4=%lu CNT4=%lu PPR=%u\r\n",
                 st[fan_get_state()],
                 fan_get_current_duty(),
                 fan_get_target_duty(),
                 (unsigned long)pulse_get_freq_hz(1),
                 (unsigned long)pulse_get_rpm(1, s_pulses_per_rev),
                 (unsigned long)pulse_get_pulse_count(1),
                 (unsigned long)pulse_get_freq_hz(2),
                 (unsigned long)pulse_get_rpm(2, s_pulses_per_rev),
                 (unsigned long)pulse_get_pulse_count(2),
                 (unsigned long)pulse_get_freq_hz(3),
                 (unsigned long)pulse_get_rpm(3, s_pulses_per_rev),
                 (unsigned long)pulse_get_pulse_count(3),
                 (unsigned long)pulse_get_freq_hz(4),
                 (unsigned long)pulse_get_rpm(4, s_pulses_per_rev),
                 (unsigned long)pulse_get_pulse_count(4),
                 (unsigned)s_pulses_per_rev);
    (void)n;
    cdc_reply(buf);
    return;
  }
  /* HELP */
  if (str_ieq("HELP", line, len))
  {
    cdc_reply("CMD: START / S0-100 / STOP / STATUS / PPRn(每转脉冲数) / HELP\r\n");
    return;
  }
  /* Sxx 设置占空比 */
  if (len >= 2 && (line[0] == 'S' || line[0] == 's'))
  {
    /* 解析 S 后的数字 */
    char numbuf[8];
    uint16_t i;
    if (len - 1 >= sizeof(numbuf))
    {
      cdc_reply("ERR NUM_TOO_LONG\r\n");
      return;
    }
    for (i = 0; i < len - 1; i++) numbuf[i] = (char)line[i + 1];
    numbuf[len - 1] = 0;
    /* 检查全数字 */
    for (i = 0; i < len - 1; i++)
    {
      if (numbuf[i] < '0' || numbuf[i] > '9')
      {
        cdc_reply("ERR BAD_NUM\r\n");
        return;
      }
    }
    int val = atoi(numbuf);
    if (val < 0) val = 0;
    if (val > 100) val = 100;
    if (val == 0)
    {
      fan_stop();
      cdc_reply("OK S0 -> STOP\r\n");
    }
    else
    {
      fan_set_target((uint8_t)val);
      static char buf[32];
      snprintf(buf, sizeof(buf), "OK S%u TARGET=%u\r\n",
               (unsigned)val, (unsigned)val);
      cdc_reply(buf);
    }
    return;
  }
  /* PPRn 设置每转脉冲数（FG 测速换算 RPM 用） */
  if (len >= 4 && (line[0] == 'P' || line[0] == 'p')
                && (line[1] == 'P' || line[1] == 'p')
                && (line[2] == 'R' || line[2] == 'r'))
  {
    char numbuf[6];
    uint16_t i;
    if (len - 3 >= sizeof(numbuf))
    {
      cdc_reply("ERR NUM_TOO_LONG\r\n");
      return;
    }
    for (i = 0; i < len - 3; i++)
    {
      numbuf[i] = (char)line[i + 3];
      if (numbuf[i] < '0' || numbuf[i] > '9')
      {
        cdc_reply("ERR BAD_NUM\r\n");
        return;
      }
    }
    numbuf[len - 3] = 0;
    int ppr = atoi(numbuf);
    if (ppr < 1) ppr = 1;
    if (ppr > 255) ppr = 255;
    s_pulses_per_rev = (uint8_t)ppr;
    static char pbuf[32];
    snprintf(pbuf, sizeof(pbuf), "OK PPR=%u\r\n", (unsigned)ppr);
    cdc_reply(pbuf);
    return;
  }
  /* 未知指令 */
  cdc_reply("ERR UNKNOWN CMD\r\n");
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
