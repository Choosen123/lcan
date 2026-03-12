#include "cdc.h"
#include "usbd_ioreq.h"

__ALIGN_BEGIN uint8_t cdc_tx_buffer[2048] __ALIGN_END;
__ALIGN_BEGIN uint8_t cdc_rx_buffer[2048] __ALIGN_END;

static uint8_t line_coding_data[7];

static USBD_CDC_HandleTypeDef *g_cdc;

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

uint8_t CDC_Setup_Request(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
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
