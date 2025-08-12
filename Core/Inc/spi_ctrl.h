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

#ifndef _SPI_CTRL_H
#define _SPI_CTRL_H


#include "stdint.h"
#include "string.h"
#include "stm32g0xx_hal.h"
#include "esp32_interface.h"
//#include "msg.h"
#include "events.h"

#define SPI_TIMEOUT 1000 // in ms

typedef enum spi_ctrl_state_e {
	SPI_CTRL_IDLE = 0x01,        //1
	SPI_CTRL_SENDING = 0x02,	//2
	SPI_CTRL_RECEIVING = 0x04,	//3
	SPI_CTRL_TX_TIMEOUT = 0x08,	//4
	SPI_CTRL_ERROR = 0x10,		// 5
	SPI_CTRL_MSG_RECEIVED = 0x20, //6
	SPI_CTRL_MSG_SENT = 0x40,	//7
	SPI_CTRL_RX_TIMEOUT = 0x80 // 8
} spi_ctrl_state_t;


void spi_ctrl_receive_abort();
void spi_ctrl_loop();
uint8_t spi_ctrl_get_rx_len();
HAL_StatusTypeDef spi_ctrl_receive(uint8_t* data, size_t length);
//HAL_StatusTypeDef spi_ctrl_send_cmd(spi_cmd_esp_t cmd_esp, spi_cmd_resp_t cmd);
HAL_StatusTypeDef spi_ctrl_send(uint8_t* data, size_t length);

//static void ADC_Set_Single_Acq();
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi);
uint8_t spi_ctrl_msg_received();
uint8_t spi_ctrl_msg_sent();
uint8_t spi_ctrl_isIdle();



//typedef enum uint32_t {
//	EVENT_SPI_MSG_RECEIVED,
//	EVENT_SPI_MSG_SENT,
//	EVENT_SPI_MSG_RX_TIMEOUT,
//	EVENT_SPI_MSG_TX_TIMEOUT
//} spi_events_t;


#endif
