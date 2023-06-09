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
