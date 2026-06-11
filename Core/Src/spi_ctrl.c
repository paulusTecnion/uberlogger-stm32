/*
 * MIT License
 *
 * Copyright (c) 2025 Tecnion Technologies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * Watchdog timers:
 *   TIM14 — SPI TX-timeout watchdog.  Started in spi_ctrl_send() when a DMA
 *            transmit is kicked off.  HAL_SPI_TxCpltCallback() clears it on
 *            normal completion (disables UIE, resets CNT).  If the timer fires
 *            before completion, spi_ctrl_loop() detects SPI_CTRL_TX_TIMEOUT,
 *            aborts the DMA transfer, and returns the state machine to IDLE.
 *
 *   TIM16 — SPI RX-timeout watchdog.  Started in spi_ctrl_receive() when a
 *            DMA receive is kicked off.  HAL_SPI_RxCpltCallback() clears it on
 *            normal completion (disables UIE, resets CNT).  If the timer fires
 *            before completion, spi_ctrl_loop() detects SPI_CTRL_RX_TIMEOUT,
 *            aborts the DMA transfer, and returns the state machine to IDLE.
 */

#include <spi_ctrl.h>
#include "main.h"

uint8_t spi_ctrl_state = SPI_CTRL_IDLE;
static uint8_t _curr_spi_state = SPI_CTRL_IDLE, _next_spi_state = SPI_CTRL_IDLE;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;

/* Moved verbatim from main.c's HAL_TIM_PeriodElapsedCallback (TIM14/TIM16
 * branches). Invoked by the single weak callback that now lives in
 * acquisition.c, so each timer branch stays with its owning module. */
void spi_ctrl_on_timeout_tick(TIM_HandleTypeDef *htim)
{
	if (htim == &htim14)
	{
		// Disable interrupt
//		TIM14->DIER &= ~TIM_DIER_UIE;
//		CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);

		TIM14->CNT = 0;
		// Indicate timeout
		SET_BIT(spi_ctrl_state, SPI_CTRL_TX_TIMEOUT);
	}

	if (htim == &htim16)
	{
//		TIM16->DIER &= ~TIM_DIER_UIE;
//		CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);
		//		CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);
		TIM16->CNT = 0;
		// Indicate timeout
		SET_BIT(spi_ctrl_state, SPI_CTRL_RX_TIMEOUT);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// Clear timeout interrupt
	CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);
	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, RESET);
	CLEAR_BIT(spi_ctrl_state, SPI_CTRL_SENDING);

	TIM14->CNT = 0;
	_next_spi_state = SPI_CTRL_IDLE;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);
	TIM16->CNT = 0;
	CLEAR_BIT(spi_ctrl_state, SPI_CTRL_RECEIVING);
	SET_BIT(spi_ctrl_state, SPI_CTRL_MSG_RECEIVED);
}


HAL_StatusTypeDef spi_ctrl_receive(uint8_t* data, size_t length)
{
	HAL_StatusTypeDef errorcode;
	if (spi_ctrl_isIdle())
	{
		errorcode = HAL_SPI_Receive_DMA(&hspi1, data, length);

		if (errorcode == HAL_OK)
		{
			SET_BIT(spi_ctrl_state, SPI_CTRL_RECEIVING);
			// clear interrupt flag
			CLEAR_BIT(TIM16->SR,TIM_SR_UIF);
			TIM16->CNT = 0;
			// Enable interrupt
			SET_BIT(TIM16->DIER,TIM_DIER_UIE);

			return errorcode;
		}


		HAL_SPI_DMAStop(&hspi1);
		return errorcode;
	}
	return HAL_BUSY;
}


HAL_StatusTypeDef spi_ctrl_send(uint8_t* data, size_t length)
{
	HAL_StatusTypeDef errorcode;

	if (spi_ctrl_isIdle())
	{
		errorcode = HAL_SPI_Transmit_DMA(&hspi1, data, length);

		if (errorcode == HAL_OK)
		{
			HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, SET);
			SET_BIT(spi_ctrl_state, SPI_CTRL_SENDING);

			// clear interrupt flag
			CLEAR_BIT(TIM14->SR,TIM_SR_UIF);
			TIM14->CNT = 0;
			// Enable interrupt
			SET_BIT(TIM14->DIER,TIM_DIER_UIE);

			return errorcode;
		}

		HAL_SPI_DMAStop(&hspi1);
		return errorcode;
	}

	return HAL_BUSY;

}

void spi_ctrl_cancel_receive(void)
{
	/* Drop a completed-but-unconsumed message: the caller is abandoning
	 * command processing to start streaming (trigger wins; the ESP32 retries
	 * its command on its own timeout). */
	CLEAR_BIT(spi_ctrl_state, SPI_CTRL_MSG_RECEIVED);

	if (READ_BIT(spi_ctrl_state, SPI_CTRL_RECEIVING))
	{
		/* Mirror of the RX-timeout teardown in spi_ctrl_loop(). */
		CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);
		TIM16->CNT = 0;
		HAL_SPI_DMAStop(&hspi1);
		CLEAR_BIT(spi_ctrl_state, SPI_CTRL_RX_TIMEOUT);
		CLEAR_BIT(spi_ctrl_state, SPI_CTRL_RECEIVING);
	}
	_next_spi_state = SPI_CTRL_IDLE;
	_curr_spi_state = SPI_CTRL_IDLE;
}

uint8_t spi_ctrl_msg_received()
{
	uint8_t result = READ_BIT(spi_ctrl_state, SPI_CTRL_MSG_RECEIVED);
	if (result)
	{
		CLEAR_BIT(spi_ctrl_state, SPI_CTRL_MSG_RECEIVED);
		_curr_spi_state = SPI_CTRL_IDLE;
	}


	return result;
}

uint8_t spi_ctrl_isIdle()
{
	if (_curr_spi_state == SPI_CTRL_IDLE)
	{
		return 1;
	} else {
		return 0;
	}
}

void spi_ctrl_loop()
{
	switch (_curr_spi_state)
	{
		case SPI_CTRL_IDLE:
			if (READ_BIT(spi_ctrl_state, SPI_CTRL_SENDING)){
				_next_spi_state = SPI_CTRL_SENDING;
			} else if (READ_BIT(spi_ctrl_state, SPI_CTRL_RECEIVING)){
				_next_spi_state = SPI_CTRL_RECEIVING;
			}
			break;

		case SPI_CTRL_SENDING:
			if (READ_BIT(spi_ctrl_state, SPI_CTRL_TX_TIMEOUT))
			{
				// clear timeout counter
				TIM14->CNT = 0;
				// Disable interrupt
				CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);
				HAL_SPI_DMAStop(&hspi1);
				// Not necessary for receiving, but no harm in making data_rdy low
				HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, RESET);

				// this is assuming we can only do send or receive simultaneously!
				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_TX_TIMEOUT);
				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_SENDING);
				_next_spi_state = SPI_CTRL_IDLE;

			}


			break;

		case SPI_CTRL_RECEIVING:
			// Intentionally, we don't check for received messages here, but
			// in spi_ctrl_msg_received, since we want to deal with that asap.
			if (READ_BIT(spi_ctrl_state, SPI_CTRL_RX_TIMEOUT))
			{
				// clear timeout counter
				TIM16->CNT = 0;
				// Disable interrupt
				CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);

				HAL_SPI_DMAStop(&hspi1);

				// this is assuming we can only do send or receive simultaneously!
				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_RX_TIMEOUT);
				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_RECEIVING);
				_next_spi_state = SPI_CTRL_IDLE;
			}
			break;

	}

	if (_next_spi_state != _curr_spi_state)
	{
		_curr_spi_state = _next_spi_state;
	}



}



