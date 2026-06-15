/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "can_common.h"
#include "led.h"
#include "timer.h"
#include "usbd_gs_can.h"
#include <stdint.h>

#ifndef CONFIG_CANFD
const struct gs_device_bt_const_extended CAN_btconst_ext;
#endif

USBD_HandleTypeDef* get_usb_handle(void);
void CAN_RestartReceiveFromGost(USBD_HandleTypeDef *pdev, USBD_GS_CAN_HandleTypeDef *hcan){
	if(hcan->from_host_buf == NULL){
		USBD_GS_CAN_ReceiveFromHost(pdev);
	}
}

int can_check_bittiming(const struct can_bittiming_const *btc,
						const struct gs_device_bittiming *timing)
{
	const uint32_t tseg1 = timing->prop_seg + timing->phase_seg1;

	if (tseg1 < btc->tseg1_min ||
		tseg1 > btc->tseg1_max ||
		timing->phase_seg2 < btc->tseg2_min ||
		timing->phase_seg2 > btc->tseg2_max ||
		timing->sjw > btc->sjw_max ||
		timing->brp < btc->brp_min ||
		timing->brp > btc->brp_max)
		return -1;

	return 0;
}

void CAN_SendFrame(USBD_GS_CAN_HandleTypeDef *hcan, can_data_t *channel)
{
	struct gs_host_frame_object *frame_object;

	bool was_irq_enabled = disable_irq();
	frame_object = list_first_entry_or_null(&channel->list_from_host,
											struct gs_host_frame_object,
											list);
	if (!frame_object) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&frame_object->list);
	restore_irq(was_irq_enabled);

	struct gs_host_frame *frame = &frame_object->frame;

	if (!can_send(channel, frame)) {
		list_add_locked(&frame_object->list, &channel->list_from_host);
		return;
	}

	uint32_t frame_time_us = gs_frame_time_us(frame, channel->nominal_bitrate, channel->data_bitrate);
	channel->frame_time_acc += frame_time_us;

	// Echo sent frame back to host
	frame->reserved = 0x0;
	if (IS_ENABLED(CONFIG_CANFD) && frame->flags & GS_CAN_FLAG_FD)
		frame->canfd_ts->timestamp_us = timer_get();
	else
		frame->classic_can_ts->timestamp_us = timer_get();

	list_add_tail_locked(&frame_object->list, &hcan->list_to_host);

	led_indicate_trx(&channel->leds, LED_TX);
}

void CAN_ReceiveFrame(USBD_GS_CAN_HandleTypeDef *hcan, can_data_t *channel)
{
	struct gs_host_frame_object *frame_object;

	if (!can_is_rx_pending(channel)) {
		return;
	}

	bool was_irq_enabled = disable_irq();
	frame_object = list_first_entry_or_null(&hcan->list_frame_pool,
											struct gs_host_frame_object,
											list);
	if (!frame_object) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&frame_object->list);
	restore_irq(was_irq_enabled);

	struct gs_host_frame *frame = &frame_object->frame;

	if (!can_receive(channel, frame)) {
		list_add_tail_locked(&frame_object->list, &hcan->list_frame_pool);
		CAN_RestartReceiveFromGost(get_usb_handle(), hcan);
		return;
	}

	uint32_t frame_time_us = gs_frame_time_us(frame, channel->nominal_bitrate, channel->data_bitrate);
	channel->frame_time_acc += frame_time_us;

	frame->echo_id = 0xFFFFFFFF; // not an echo frame
	frame->reserved = 0;

	list_add_tail_locked(&frame_object->list, &hcan->list_to_host);

	led_indicate_trx(&channel->leds, LED_RX);
}

// If there are frames to receive, don't report any error frames. The
// best we can localize the errors to is "after the last successfully
// received frame", so wait until we get there. LEC will hold some error
// to report even if multiple pass by.
void CAN_HandleError(USBD_GS_CAN_HandleTypeDef *hcan, can_data_t *channel)
{
	struct gs_host_frame_object *frame_object;

	if (can_is_rx_pending(channel)) {
		return;
	}

	uint32_t can_err = can_get_error_status(channel);

	bool was_irq_enabled = disable_irq();
	frame_object = list_first_entry_or_null(&hcan->list_frame_pool,
											struct gs_host_frame_object,
											list);
	if (!frame_object) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&frame_object->list);
	restore_irq(was_irq_enabled);

	struct gs_host_frame *frame = &frame_object->frame;
	frame->classic_can_ts->timestamp_us = timer_get();
	frame->channel = channel->nr;

	if (can_parse_error_status(channel, frame, can_err)) {
		list_add_tail_locked(&frame_object->list, &hcan->list_to_host);
	} else {
		list_add_tail_locked(&frame_object->list, &hcan->list_frame_pool);
		CAN_RestartReceiveFromGost(get_usb_handle(), hcan);
	}
}

uint32_t gs_frame_time_us(struct gs_host_frame *frame, uint32_t nominal, uint32_t data)
{
    static const uint8_t dlc2len[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    if (nominal == 0) return 0;

    uint8_t len = dlc2len[frame->can_dlc & 0x0F];
    bool ext = frame->can_id & CAN_EFF_FLAG;
    bool fd  = frame->flags & GS_CAN_FLAG_FD;
    bool brs = frame->flags & GS_CAN_FLAG_BRS;

    if (!fd)
    {
        // --- Classic CAN ---
        // 基础位：SOF(1)+ID(11/29)+RTR(1)+IDE(1)+r0(1)+DLC(4)+CRC(15)+CRCDel(1)+ACK(1)+ACKDel(1)+EOF(7)
        // 标准帧约 44 bits, 扩展帧约 64 bits (不计 IFS)
        uint32_t base_bits = (ext ? 64 : 44);
        uint32_t payload_bits = len * 8;

        // 位填充：PCAN 等工具常用 1.1 左右的经验值，或者取 (total * 10 / 9)
        // 这里推荐使用 1.06 (约 17/16)，更接近真实表现
        uint32_t total_bits = (base_bits + payload_bits);
        total_bits = (total_bits * 17) / 16;

        total_bits += 3; // 加上 3 bits IFS (Inter-frame Space, 不参与填充)

        return (uint64_t)total_bits * 1000000ULL / nominal;
    }
    else
    {
        // --- CAN FD ---
        // 仲裁段 (Nominal Rate): SOF(1)+ID(11/29)+SRR(1)+IDE(1)+FDF(1)+res(1)+BRS(1)+ESI(1)
        // 标准帧仲裁段约 18 bits, 扩展帧约 36 bits
        uint32_t arb_bits = (ext ? 36 : 18);

        // 数据段 (Data Rate): DLC(4)+Data(len*8)+CRC+CRC_Stuff+ACK+EOF+IFS
        // CAN FD 的 CRC 段有固定位填充：CRC17(28bits, len<=16) 或 CRC21(33bits, len>16)
        uint32_t data_bits = (len * 8);
        if (len <= 16) {
            data_bits += 28; // DLC(4) + CRC(17) + FixedStuff(4+2+1) + ACK...
        } else {
            data_bits += 33; // DLC(4) + CRC(21) + FixedStuff(5+2+1) + ACK...
        }

        // 位填充系数：CAN FD 数据段由于已经有了固定填充，额外填充概率降低
        // 经验值取 1.03 (约 33/32)
        data_bits = (data_bits * 33) / 32;

        uint32_t time_us = (uint64_t)arb_bits * 1000000ULL / nominal;
        time_us += (uint64_t)data_bits * 1000000ULL / (brs ? data : nominal);

        return time_us;
    }
}
// uint32_t gs_frame_time_us(struct gs_host_frame *frame,
//                           uint32_t nominal,
//                           uint32_t data)
// {
//     static const uint8_t dlc2len[16] =
//     {
//          0,1,2,3,
//          4,5,6,7,
//          8,12,16,20,
//          24,32,48,64
//     };
//     if (nominal == 0) return 0;

//     uint8_t len = dlc2len[frame->can_dlc & 0x0F];

//     bool ext = frame->can_id & CAN_EFF_FLAG;
//     bool fd  = frame->flags & GS_CAN_FLAG_FD;
//     bool brs = frame->flags & GS_CAN_FLAG_BRS;

//     if (brs && data == 0) return 0;

//     if (!fd)
//     {
//         uint32_t bits = (ext ? 67 : 47) + len * 8;
//         bits = bits * 12 / 10;

//         return (uint64_t)bits * 1000000ULL / nominal;
//     }

//     uint32_t arb = (ext ? 67 : 47);
//     arb = arb * 12 / 10;

//     uint32_t payload = len * 8 + 30;
//     payload = payload * 12 / 10;

//     return
//         (uint64_t)arb * 1000000ULL / nominal +
//         (uint64_t)payload * 1000000ULL /
//         (brs ? data : nominal);
// }
