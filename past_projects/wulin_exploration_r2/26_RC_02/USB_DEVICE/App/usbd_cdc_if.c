/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USB CDC 原始字节传输层：接收回调写 RX ring；应用写 TX ring；发送完成后推进队列。应用帧由 bsp_usb.c 处理。USB 栈负责端点分包。 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "usbd_cdc.h"
#include "usb_device.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

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

/* 应用层 RX/TX 环形缓冲大小，独立于 CubeMX 端点缓冲大小。 */
#define CDC_APP_RX_RING_SIZE      4096U
#define CDC_APP_TX_RING_SIZE      4096U

/* 单次提交 USB 栈的最大连续长度；USB 栈按端点大小分包。 */
#define CDC_APP_TX_CHUNK_SIZE      512U

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
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* 应用层收发 ring 存储。 */
//static uint8_t s_rxRing[CDC_APP_RX_RING_SIZE];
//static uint8_t s_txRing[CDC_APP_TX_RING_SIZE];
uint8_t s_rxRing[CDC_APP_RX_RING_SIZE];
uint8_t s_txRing[CDC_APP_TX_RING_SIZE];

/* RX 回调写、任务读；TX 应用写、发送完成后推进读指针。 */
static volatile uint32_t s_rxW = 0U;
static volatile uint32_t s_rxR = 0U;
static volatile uint32_t s_txW = 0U;
static volatile uint32_t s_txR = 0U;

/* 已提交但尚未完成的 TX 字节数。 */
static volatile uint32_t s_txInflight = 0U;

/* 接收缓冲满时丢弃的字节数。 */
static volatile uint32_t s_rxDropBytes = 0U;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* 计算环形缓冲中已使用的字节数。 */
static uint32_t RB_Used(uint32_t w, uint32_t r, uint32_t size)
{
  return (w >= r) ? (w - r) : (size - r + w);
}

/* 预留一个槽位区分空与满。 */
static uint32_t RB_Free(uint32_t w, uint32_t r, uint32_t size)
{
  return (size - 1U) - RB_Used(w, r, size);
}

/* 向 ring 追加一字节，成功返回 1。 */
static uint8_t RB_PushByte(volatile uint32_t *w,
                           volatile uint32_t *r,
                           uint8_t *buf,
                           uint32_t size,
                           uint8_t data)
{
  uint32_t next = (*w + 1U) % size;

  if (next == *r)
  {
    return 0U; /* 缓冲已满，返回失败。 */
  }

  buf[*w] = data;
  *w = next;
  return 1U;
}

/* 从 ring 取出一字节，成功返回 1。 */
static uint8_t RB_PopByte(volatile uint32_t *w,
                          volatile uint32_t *r,
                          uint8_t *buf,
                          uint32_t size,
                          uint8_t *data)
{
  if ((w == NULL) || (r == NULL) || (buf == NULL) || (data == NULL) || (size == 0U))
  {
    return 0U;
  }

  /* 空缓冲无可读数据。 */
  if (*r == *w)
  {
    return 0U;
  }

  *data = buf[*r];
  *r = (*r + 1U) % size;
  return 1U;
}


/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  /* 初始化 USB 栈的发送缓冲。 */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0U);

  /* 初始化 USB 栈的接收缓冲。 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);

  /* 初始化应用层队列索引和诊断计数。 */
  s_rxW = 0U;
  s_rxR = 0U;
  s_txW = 0U;
  s_txR = 0U;
  s_txInflight = 0U;
  s_rxDropBytes = 0U;

  /* 重新使能 OUT 端点接收。 */
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
	while(0){// 未启用的旧 CDC 控制模板。
//  switch(cmd)
//  {
//  case CDC_SEND_ENCAPSULATED_COMMAND:

//    break;

//  case CDC_GET_ENCAPSULATED_RESPONSE:

//    break;

//  case CDC_SET_COMM_FEATURE:

//    break;

//  case CDC_GET_COMM_FEATURE:

//    break;

//  case CDC_CLEAR_COMM_FEATURE:

//    break;

//  /*******************************************************************************/
//  /* Line Coding Structure                                                       */
//  /*-----------------------------------------------------------------------------*/
//  /* Offset | Field       | Size | Value  | Description                          */
//  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
//  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
//  /*                                        0 - 1 Stop bit                       */
//  /*                                        1 - 1.5 Stop bits                    */
//  /*                                        2 - 2 Stop bits                      */
//  /* 5      | bParityType |  1   | Number | Parity                               */
//  /*                                        0 - None                             */
//  /*                                        1 - Odd                              */
//  /*                                        2 - Even                             */
//  /*                                        3 - Mark                             */
//  /*                                        4 - Space                            */
//  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
//  /*******************************************************************************/
//  case CDC_SET_LINE_CODING:

//    break;

//  case CDC_GET_LINE_CODING:

//    break;

//  case CDC_SET_CONTROL_LINE_STATE:

//    break;

//  case CDC_SEND_BREAK:

//    break;

//  default:
//    break;
//  }

//  return (USBD_OK);
 }
  (void)length;

  switch (cmd)
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

    case CDC_SET_LINE_CODING:
      /* 虚拟串口线编码设置不改变底层 UART 配置。 */
      break;

    case CDC_GET_LINE_CODING:
      /* 返回虚拟串口默认线编码。 */
      /* 115200 baud，8 数据位、无校验、1 停止位。 */
      pbuf[0] = 0x00;
      pbuf[1] = 0xC2;
      pbuf[2] = 0x01;
      pbuf[3] = 0x00; /* 115200 = 0x0001C200 */
      pbuf[4] = 0x00; /* 1 stop bit */
      pbuf[5] = 0x00; /* no parity */
      pbuf[6] = 0x08; /* 8 data bits */
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* DTR/RTS 控制请求当前不控制机构或串口。 */
      break;

    case CDC_SEND_BREAK:
      break;

    default:
      break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
//  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
//  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
//  return (USBD_OK);
	uint32_t i;

  /* 接收回调将本次 USB 数据逐字节写入 RX ring。 */
  for (i = 0U; i < *Len; i++)
  {
    if (RB_PushByte(&s_rxW, &s_rxR, s_rxRing, CDC_APP_RX_RING_SIZE, Buf[i]) == 0U)
    {
      /* RX ring 满：丢弃当前字节并累计诊断计数。 */
      s_rxDropBytes++;
    }
  }

  /* 恢复接收缓冲并继续接收下一包。 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, Buf);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
//  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
//  if (hcdc->TxState != 0){
//    return USBD_BUSY;
//  }
//  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
//  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS)
  uint32_t i;
  uint32_t free_space;

  if ((Buf == NULL) || (Len == 0U))
  {
    return USBD_OK;
  }

  /* 空间不足则拒绝整次写入，不提交半包。 */
  free_space = RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if ((uint32_t)Len > free_space)
  {
    return USBD_BUSY;
  }

  /* 数据复制入 TX ring。 */
  for (i = 0U; i < (uint32_t)Len; i++)
  {
    (void)RB_PushByte(&s_txW, &s_txR, s_txRing, CDC_APP_TX_RING_SIZE, Buf[i]);
  }

  /* 尝试启动发送；忙时由后续发送完成路径继续推进。 */
  CDC_App_TxTask();

  return USBD_OK;
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  /* 推进 TX ring 读指针，清除 inflight，提交下一块 */
  s_txR    = (s_txR + s_txInflight) % CDC_APP_TX_RING_SIZE;
  s_txInflight = 0U;
  CDC_App_TxTask();

  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* 应用层字节队列接口。 */
/* 返回 RX ring 当前可读字节数。 */
uint32_t CDC_App_Available(void)
{
  return RB_Used(s_rxW, s_rxR, CDC_APP_RX_RING_SIZE);
}

/* 返回 TX ring 当前空闲字节数。 */
uint32_t CDC_App_TxFree(void)
{
  return RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
}

/* 返回 RX ring 累计丢弃字节数。 */
uint32_t CDC_App_GetRxDropped(void)
{
  return s_rxDropBytes;
}

/* 最多读取 max_len 字节，返回实际读取长度。 */
uint32_t CDC_App_Read(uint8_t *buf, uint32_t max_len)
{
  uint32_t count = 0U;
  uint8_t  data;

  if ((buf == NULL) || (max_len == 0U))
  {
    return 0U;
  }

  while (count < max_len)
  {
    if (RB_PopByte(&s_rxW, &s_rxR, s_rxRing, CDC_APP_RX_RING_SIZE, &data) == 0U)
    {
      break; /* ring 已空，结束本次读取。 */
    }

    buf[count++] = data;
  }

  return count;
}

/* 向 TX ring 写入完整数据块，成功后尝试启动发送。 */
uint8_t CDC_App_Write(const uint8_t *buf, uint32_t len)
{
  uint32_t i;
  uint32_t free_space;

  if ((buf == NULL) || (len == 0U))
  {
    return 1U;
  }

  /* 先检查全部数据能否入队，避免部分写入。 */
  free_space = RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if (len > free_space)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    (void)RB_PushByte(&s_txW, &s_txR, s_txRing, CDC_APP_TX_RING_SIZE, buf[i]);
  }

  /* 数据入队后尝试启动 USB 传输。 */
  CDC_App_TxTask();

  return 1U;
}

/* 推进 TX ring 发送；正在传输时不重复提交。 */
void CDC_App_TxTask(void)
{
  USBD_CDC_HandleTypeDef *hcdc;
  uint32_t used;
  uint32_t linear_len;

  /* USB CDC 类尚未初始化时不提交数据。 */
  if (hUsbDeviceHS.pClassData == NULL)
  {
    return;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;

  /* 存在未完成传输时等待完成回调。 */
  if (s_txInflight != 0U)
  {
    return;
  }

  /* USB 栈忙时等待下一次推进。 */
  if (hcdc->TxState != 0U)
  {
    return;
  }

  /* 队列为空时没有数据可发送。 */
  used = RB_Used(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if (used == 0U)
  {
    return;
  }

  /* 计算读指针到 ring 末端的连续可发送长度。 */
  if (s_txW > s_txR)
  {
    linear_len = s_txW - s_txR;
  }
  else
  {
    linear_len = CDC_APP_TX_RING_SIZE - s_txR;
  }

  if (linear_len > used)
  {
    linear_len = used;
  }

  /* 限制本次提交长度，保持下一段数据在 ring 中。 */
  if (linear_len > CDC_APP_TX_CHUNK_SIZE)
  {
    linear_len = CDC_APP_TX_CHUNK_SIZE;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, &s_txRing[s_txR], (uint16_t)linear_len);

  if (USBD_CDC_TransmitPacket(&hUsbDeviceHS) == USBD_OK)
  {
    /* 记录提交长度，完成回调再推进读指针。 */
    s_txInflight = linear_len;
  }
}


/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
