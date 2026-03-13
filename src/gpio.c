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

#include "board.h"
#include "config.h"
#include "gpio.h"
#include "hal_include.h"
#include "stm32g0xx_hal_gpio.h"

#ifdef TERM_Pin
static int term_state = 0;

enum gs_can_termination_state get_term(can_data_t * channel)
{
	const uint8_t nr = channel->nr;

	if (term_state & (1 << nr)) {
		return GS_CAN_TERMINATION_STATE_ON;
	} else {
		return GS_CAN_TERMINATION_STATE_OFF;
	}
}

enum gs_can_termination_state set_term(can_data_t *channel, enum gs_can_termination_state state)
{
	const uint8_t nr = channel->nr;

	if (state == GS_CAN_TERMINATION_STATE_ON) {
		term_state |= 1 << nr;
	} else {
		term_state &= ~(1 << nr);
	}

	config.termination_set(channel, state);

	return state;
}

#endif

void gpio_init(void)
{
	#ifdef BOARD_candleLightFD
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA3 PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	#endif
}
