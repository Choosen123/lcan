#include "cdc.h"
#include "stm32g0xx_hal_uart.h"
#include "usbd_core.h"
#include "usbd_def.h"
#include "usbd_gs_can.h"
#include "usbd_ioreq.h"
#include <stdint.h>
#include <string.h>

extern USBD_HandleTypeDef hUSB;

__ALIGN_BEGIN uint8_t cdc_tx_buffer[CDC_TX_BUFFER_SIZE] __ALIGN_END;	// USB发送数据缓冲区，设备发给主机存入cdc_tx_buffer，主机读取cdc_tx_buffer设备数据
__ALIGN_BEGIN uint8_t cdc_rx_buffer[CDC_RX_BUFFER_SIZE] __ALIGN_END;	// USB接收数据缓冲区，主机发给设备存入cdc_rx_buffer，设备读取cdc_rx_buffer发送到总线

volatile uint32_t cdc_rx_write_ptr = 0;		// USB接收数据写入指针，主机发给设备存入cdc_rx_buffer
volatile uint32_t cdc_rx_read_ptr = 0;		// UART DMA接收数据读取指针，设备读取cdc_rx_buffer主机数据
volatile uint32_t cdc_tx_read_ptr = 0;		// USB发送数据读取指针，写入靠dma自动完成

static uint8_t line_coding_data[7];

static USBD_CDC_HandleTypeDef *g_cdc;

// 初始化CDC接口
void CDC_Init(USBD_CDC_HandleTypeDef *hcdc, UART_HandleTypeDef *huart)
{
    g_cdc = hcdc;
    // 默认波特率等设置
    g_cdc->line_coding.bitrate = 115200;
    g_cdc->line_coding.format = 0;
    g_cdc->line_coding.paritytype = 0;
    g_cdc->line_coding.datatype = 8;

    g_cdc->RxBuffer = cdc_rx_buffer;
    g_cdc->TxBuffer = cdc_tx_buffer;
    g_cdc->tx_busy = false;
    g_cdc->rx_ptr_read = 0;

    g_cdc->huart = huart;
}

// 处理CDC类请求
uint8_t CDC_Setup_Request(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
    // USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
    USBD_CDC_HandleTypeDef *hcdc = g_cdc;

    switch(req->bRequest){
	   	case CDC_SET_LINE_CODING:
	   		USBD_CtlPrepareRx(pdev, line_coding_data, 7);
	  		break;

		case CDC_GET_LINE_CODING:
			line_coding_data[0] = (uint8_t)(hcdc->line_coding.bitrate);
			line_coding_data[1] = (uint8_t)(hcdc->line_coding.bitrate >> 8);
			line_coding_data[2] = (uint8_t)(hcdc->line_coding.bitrate >> 16);
			line_coding_data[3] = (uint8_t)(hcdc->line_coding.bitrate >> 24);
			line_coding_data[4] = hcdc->line_coding.format;
			line_coding_data[5] = hcdc->line_coding.paritytype;
			line_coding_data[6] = hcdc->line_coding.datatype;
			USBD_CtlSendData(pdev, line_coding_data, 7);
			break;

		default:
            USBD_CtlError(pdev, req);
            return USBD_FAIL;
       		break;
    }

    return USBD_OK;
}

// 设置新的串口参数
void CDC_SetLineCoding(USBD_CDC_HandleTypeDef* hcdc, const USBD_CDC_LineCodingTypeDef* coding){
	memcpy(&hcdc->line_coding, coding, sizeof(USBD_CDC_LineCodingTypeDef));
	hcdc->huart->Init.BaudRate = coding->bitrate;

	// 数据位设置
	switch(coding->datatype){
		case 7:
			hcdc->huart->Init.WordLength = UART_WORDLENGTH_7B;
			break;
		case 8:
			hcdc->huart->Init.WordLength = UART_WORDLENGTH_8B;
			break;
		default:
			hcdc->huart->Init.WordLength = UART_WORDLENGTH_9B;
			break;
	}

	// 停止位设置
	switch(coding->format){
		case 0:
			hcdc->huart->Init.StopBits = UART_STOPBITS_1;
			break;
		case 2:
			hcdc->huart->Init.StopBits = UART_STOPBITS_2;
			break;
		default:
			hcdc->huart->Init.StopBits = UART_STOPBITS_1;
			break;
	}

	// 校验位设置
	switch(coding->paritytype){
		case 0:
			hcdc->huart->Init.Parity = UART_PARITY_NONE;
			break;
		case 1:
			hcdc->huart->Init.Parity = UART_PARITY_ODD;
			break;
		case 2:
			hcdc->huart->Init.Parity = UART_PARITY_EVEN;
			break;
		default:
			hcdc->huart->Init.Parity = UART_PARITY_NONE;
			break;
	}

	HAL_RS485Ex_Init(hcdc->huart, UART_DE_POLARITY_HIGH, 1, 1);
}

// EP0接收完成回调函数
uint8_t CDC_EP0_RxReady(USBD_HandleTypeDef *pdev, USBD_CDC_HandleTypeDef *hcdc, uint8_t req){
	(void)pdev;
	switch(req){
		case CDC_SET_LINE_CODING:{
			// 将接收到的数据复制到 line_coding 结构体中
			USBD_CDC_LineCodingTypeDef new_coding;
			new_coding.bitrate = (uint32_t)line_coding_data[0] | ((uint32_t)line_coding_data[1] << 8) | ((uint32_t)line_coding_data[2] << 16) | ((uint32_t)line_coding_data[3] << 24);
			new_coding.format = line_coding_data[4];
			new_coding.paritytype = line_coding_data[5];
			new_coding.datatype = line_coding_data[6];

			CDC_SetLineCoding(hcdc, &new_coding);
			break;
		}

		default:
			break;
	}

	return USBD_OK;
}

// 数据发送完成回调函数
uint8_t CDC_DataIn_Callback(USBD_HandleTypeDef *pdev, uint8_t epnum){
	// USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
	(void)pdev;
	USBD_CDC_HandleTypeDef *hcdc = g_cdc;
	if(epnum == (CDC_ENDPOINT_DATA_IN & 0x7F)){
		// usb发送完成，清除发送忙标志
		hcdc->tx_busy = false;

		CDC_CheckAndTransmitUART(hcdc); // 检查是否有更多数据需要发送
	}
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
	return USBD_OK;
}

// 主机发给从机数据接收完成回调函数
uint8_t CDC_DataOut_Callback(USBD_HandleTypeDef *pdev, uint8_t epnum){
	// USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
	USBD_CDC_HandleTypeDef *hcdc = g_cdc;
	if(epnum == (CDC_ENDPOINT_DATA_OUT & 0x7F)){
		uint32_t rxlen = USBD_LL_GetRxDataSize(pdev, epnum);

		if(rxlen > 0){
			uint32_t space_to_end = CDC_RX_BUFFER_SIZE - cdc_rx_write_ptr;
			if(rxlen <= space_to_end){
				memcpy(&cdc_rx_buffer[cdc_rx_write_ptr], hcdc->RxBuffer, rxlen);
				cdc_rx_write_ptr += rxlen; // 更新写指针
				if(cdc_rx_write_ptr >= CDC_RX_BUFFER_SIZE){
					cdc_rx_write_ptr = 0; // 回绕到缓冲区开头
				}
			}else{
				// 如果数据超过缓冲区末尾，分两次拷贝
				memcpy(&cdc_rx_buffer[cdc_rx_write_ptr], hcdc->RxBuffer, space_to_end);
				memcpy(cdc_rx_buffer, &hcdc->RxBuffer[space_to_end], rxlen - space_to_end);
				cdc_rx_write_ptr = rxlen - space_to_end; // 更新写指针
			}

			CDC_CheckAndTransmitUART(hcdc); // 检查是否有数据需要发送到UART

			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);

			// 准备下一次接收
			USBD_LL_PrepareReceive(pdev, CDC_ENDPOINT_DATA_OUT, hcdc->RxBuffer, CDC_DATA_MAX_PACKET_SIZE);
		}
	}

	return USBD_OK;
}

void CDC_CheckAndTransmitUSB(USBD_HandleTypeDef *pdev){
	if(g_cdc->tx_busy){
		return; // 上一次发送还未完成
	}

	uint32_t current_dma_ptr = CDC_TX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(g_cdc->huart->hdmarx);
	if(cdc_tx_read_ptr == current_dma_ptr){
		return; // 没有新数据需要发送
	}

	uint32_t send_len;
	uint8_t *send_ptr = &cdc_tx_buffer[cdc_tx_read_ptr];

	if(cdc_tx_read_ptr <= current_dma_ptr){
		send_len = current_dma_ptr - cdc_tx_read_ptr;
		cdc_tx_read_ptr += send_len;
	}else{
		send_len = CDC_TX_BUFFER_SIZE - cdc_tx_read_ptr;
		cdc_tx_read_ptr = 0; // 回绕到缓冲区开头
	}

	g_cdc->tx_busy = true;
	USBD_LL_Transmit(pdev, CDC_ENDPOINT_DATA_IN, send_ptr, send_len);
}

// 检查是否有主机数据需要发送到UART，并启动DMA发送
void CDC_CheckAndTransmitUART(USBD_CDC_HandleTypeDef *hcdc){
	if(hcdc->huart->gState != HAL_UART_STATE_READY
	&& hcdc->huart->gState != HAL_UART_STATE_BUSY_RX){
		return; // UART正在忙，无法发送
	}

	uint32_t len_to_send;
	if(cdc_rx_write_ptr >=cdc_rx_read_ptr){
		len_to_send = cdc_rx_write_ptr - cdc_rx_read_ptr;
	}else{
		len_to_send = CDC_RX_BUFFER_SIZE - cdc_rx_read_ptr; // 发送到缓冲区末尾
	}

	if(len_to_send > 0){
		HAL_UART_Transmit_DMA(hcdc->huart, &cdc_rx_buffer[cdc_rx_read_ptr], len_to_send);
	}


}


// USART3中断处理函数，处理空闲中断以触发数据发送
void USART3_4_5_6_LPUART1_IRQHandler(void){
	// 串口空闲中断触发，说明总线上没有数据了，可以检查是否有数据需要发送给主机
	if(__HAL_UART_GET_FLAG(g_cdc->huart, UART_FLAG_IDLE)){
		__HAL_UART_CLEAR_IDLEFLAG(g_cdc->huart);
		CDC_CheckAndTransmitUSB(&hUSB);
	}

	HAL_UART_IRQHandler(g_cdc->huart);
}

// UART发送完成回调函数，继续检查是否有更多数据需要发送到总线上
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == g_cdc->huart){
		// 更新读取指针
		uint32_t sent_len = huart->TxXferSize;
		cdc_rx_read_ptr += sent_len;
		if(cdc_rx_read_ptr >= CDC_RX_BUFFER_SIZE){
			cdc_rx_read_ptr = 0; // 回绕到缓冲区开头
		}
		CDC_CheckAndTransmitUART(g_cdc);
	}
}

// void CDC_Process(USBD_HandleTypeDef *pdev, USBD_CDC_HandleTypeDef *hcdc){
// }
