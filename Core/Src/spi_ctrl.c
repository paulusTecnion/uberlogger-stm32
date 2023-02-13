#include <spi_ctrl.h>
#include "main.h"

uint8_t spi_ctrl_state = SPI_CTRL_IDLE;
uint8_t _curr_spi_state = SPI_CTRL_IDLE, _next_spi_state = SPI_CTRL_IDLE;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, RESET);
	CLEAR_BIT(spi_ctrl_state, SPI_CTRL_SENDING);
	// Reset counter
	TIM14->CNT = 0;
	SET_BIT(spi_ctrl_state, SPI_CTRL_MSG_SENT);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
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
			TIM14->CNT = 0;
			HAL_TIM_Base_Start_IT(&htim14);
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
			TIM14->CNT = 0;
			HAL_TIM_Base_Start(&htim14);
			return errorcode;
		}

		HAL_SPI_DMAStop(&hspi1);
		return errorcode;
	}

	return HAL_BUSY;

}

void spi_ctrl_receive_abort()
{

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

// don't use this function.
uint8_t spi_ctrl_msg_sent()
{
		uint8_t result = READ_BIT(spi_ctrl_state, SPI_CTRL_MSG_SENT);
		if (result)
		{
			CLEAR_BIT(spi_ctrl_state, SPI_CTRL_MSG_SENT);
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
			if (READ_BIT(spi_ctrl_state, SPI_CTRL_MSG_SENT))
			{
				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_MSG_SENT);
				_next_spi_state = SPI_CTRL_IDLE;
			}
			else
			if (READ_BIT(spi_ctrl_state, SPI_CTRL_TIMEOUT))
			{
			// stop timeout timer
				HAL_TIM_Base_Stop_IT(&htim14);
				// clear timeout counter
				TIM14->CNT = 0;
				HAL_SPI_DMAStop(&hspi1);
				// Not necessary for receiving, but no harm in making data_rdy low
				HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, RESET);

				// this is assuming we can only do send or receive simultaneously!
				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_TIMEOUT);
				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_SENDING);
				_next_spi_state = SPI_CTRL_IDLE;

			}


			break;

		case SPI_CTRL_RECEIVING:
//			if (READ_BIT(spi_ctrl_state, SPI_CTRL_MSG_RECEIVED))
//			{
//				CLEAR_BIT(spi_ctrl_state, SPI_CTRL_MSG_RECEIVED);
//				_next_spi_state = SPI_CTRL_IDLE;
//			}
//			else
			if (READ_BIT(spi_ctrl_state, SPI_CTRL_TIMEOUT))
			{
				// stop timeout timer
					HAL_TIM_Base_Stop_IT(&htim14);
					// clear timeout counter
					TIM14->CNT = 0;
					HAL_SPI_DMAStop(&hspi1);

					// this is assuming we can only do send or receive simultaneously!
					CLEAR_BIT(spi_ctrl_state, SPI_CTRL_TIMEOUT);
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



