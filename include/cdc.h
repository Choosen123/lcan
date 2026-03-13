#pragma once
#include "stdint.h"
#include "stdbool.h"
#include "stm32g0xx_hal.h"
#include "usbd_def.h"

#define CDC_ENDPOINT_CMD        0x83
#define CDC_ENDPOINT_DATA_IN    0x84
#define CDC_ENDPOINT_DATA_OUT   0x04

#define CDC_CTRL_INTERFACE_NUM 2
#define CDC_DATA_INTERFACE_NUM 2


#define CDC_CMD_MAX_PCAKET_SIZE 8
#define CDC_DATA_MAX_PACKET_SIZE 64

#define CDC_RX_BUFFER_SIZE 2048
#define CDC_TX_BUFFER_SIZE 2048

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01U
#define CDC_SET_COMM_FEATURE                        0x02U
#define CDC_GET_COMM_FEATURE                        0x03U
#define CDC_CLEAR_COMM_FEATURE                      0x04U
#define CDC_SET_LINE_CODING                         0x20U
#define CDC_GET_LINE_CODING                         0x21U
#define CDC_SET_CONTROL_LINE_STATE                  0x22U
#define CDC_SEND_BREAK                              0x23U

extern __ALIGN_BEGIN uint8_t cdc_tx_buffer[CDC_TX_BUFFER_SIZE] __ALIGN_END;
extern __ALIGN_BEGIN uint8_t cdc_rx_buffer[CDC_RX_BUFFER_SIZE] __ALIGN_END;

typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} USBD_CDC_LineCodingTypeDef;

typedef struct _USBD_CDC_Itf
{
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
} USBD_CDC_ItfTypeDef;


typedef struct
{
  USBD_CDC_LineCodingTypeDef line_coding;

  uint8_t *RxBuffer;
  uint8_t *TxBuffer;

  UART_HandleTypeDef *huart;

  GPIO_TypeDef *de_port;
  uint16_t de_pin;

  volatile bool tx_busy;
  volatile uint32_t rx_ptr_read;

} USBD_CDC_HandleTypeDef;

void CDC_Init(USBD_CDC_HandleTypeDef *hcdc, UART_HandleTypeDef *huart);

uint8_t CDC_Setup_Request(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
void CDC_SetLineCoding(USBD_CDC_HandleTypeDef* hcdc, const USBD_CDC_LineCodingTypeDef* coding);
uint8_t CDC_EP0_RxReady(USBD_HandleTypeDef *pdev, USBD_CDC_HandleTypeDef *hcdc, uint8_t req);
uint8_t CDC_DataIn_Callback(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t CDC_DataOut_Callback(USBD_HandleTypeDef *pdev, uint8_t epnum);
void CDC_CheckAndTransmitUSB(USBD_HandleTypeDef *pdev);
void CDC_CheckAndTransmitUART(USBD_CDC_HandleTypeDef *hcdc);
