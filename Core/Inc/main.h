/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "esp32_interface.h"
#include "events.h"
//#include "msg.h"

#include "config.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


enum  {
	MAIN_IDLE = 0x01,
	MAIN_CONFIG,
	MAIN_LOGGING,
	MAIN_SINGLE_SHOT,
	MAIN_SINGLE_SHOT_AWAIT_RESULT,
	MAIN_ERROR
};



typedef struct {
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t padding1;
	uint8_t padding2;
	uint32_t subseconds;
} s_date_time_t;



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */



void ADC_Reinit();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIGITAL_IN_0_Pin GPIO_PIN_10
#define DIGITAL_IN_0_GPIO_Port GPIOB
#define DIGITAL_IN_1_Pin GPIO_PIN_11
#define DIGITAL_IN_1_GPIO_Port GPIOB
#define DIGITAL_IN_2_Pin GPIO_PIN_12
#define DIGITAL_IN_2_GPIO_Port GPIOB
#define DIGITAL_IN_3_Pin GPIO_PIN_13
#define DIGITAL_IN_3_GPIO_Port GPIOB
#define DIGITAL_IN_4_Pin GPIO_PIN_14
#define DIGITAL_IN_4_GPIO_Port GPIOB
#define DIGITAL_IN_5_Pin GPIO_PIN_15
#define DIGITAL_IN_5_GPIO_Port GPIOB
#define AIN_RANGE_SELECT_CLK_Pin GPIO_PIN_0
#define AIN_RANGE_SELECT_CLK_GPIO_Port GPIOD
#define AIN_RANGE_SELECT_CLR_Pin GPIO_PIN_1
#define AIN_RANGE_SELECT_CLR_GPIO_Port GPIOD
#define AIN_PULLUP_SELECT_CLK_Pin GPIO_PIN_2
#define AIN_PULLUP_SELECT_CLK_GPIO_Port GPIOD
#define AIN_PULLUP_SELECT_CLR_Pin GPIO_PIN_3
#define AIN_PULLUP_SELECT_CLR_GPIO_Port GPIOD
#define STM_DATA_RDY_Pin GPIO_PIN_6
#define STM_DATA_RDY_GPIO_Port GPIOB
#define STM_ADC_EN_Pin GPIO_PIN_7
#define STM_ADC_EN_GPIO_Port GPIOB
#define STM_ADC_EN_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
// WARNING: DO NOT CHANGE THE NEXT LINES UNLESS YOU KNOW WHAT YOU ARE DOING.
// BYTES NEED TO BE 4 BYTES ALIGNED!
#define DATA_LINES_PER_SPI_TRANSACTION  70
#define ADC_VALUES_PER_SPI_TRANSACTION  DATA_LINES_PER_SPI_TRANSACTION*8 // Number of ADC uint16_t per transaction. This is 5 times 480 ADC values
#define ADC_BYTES_PER_SPI_TRANSACTION ADC_VALUES_PER_SPI_TRANSACTION*2
#define GPIO_BYTES_PER_SPI_TRANSACTION  DATA_LINES_PER_SPI_TRANSACTION*1
#define TIME_BYTES_PER_SPI_TRANSACTION  DATA_LINES_PER_SPI_TRANSACTION*12
#define START_STOP_NUM_BYTES            2


// Number of bytes when receiving data from the STM
#define STM_SPI_BUFFERSIZE_DATA_TX      (ADC_BYTES_PER_SPI_TRANSACTION + GPIO_BYTES_PER_SPI_TRANSACTION + TIME_BYTES_PER_SPI_TRANSACTION + START_STOP_NUM_BYTES)
#define STM_DATA_BUFFER_SIZE_PER_TRANSACTION (ADC_BYTES_PER_SPI_TRANSACTION + GPIO_BYTES_PER_SPI_TRANSACTION + TIME_BYTES_PER_SPI_TRANSACTION)

//#define STM_SPI_BUFFERSIZE_DATA_TX 1020
#define ADC_BUFFERSIZE_SAMPLES ADC_VALUES_PER_SPI_TRANSACTION*2
#define ADC_BUFFERSIZE_BYTES ADC_BUFFERSIZE_SAMPLES*2
#define GPIO_IO_BUFFERIZE_BYTES GPIO_BYTES_PER_SPI_TRANSACTION*2
#define TIME_BUFFERSIZE_BYTES TIME_BYTES_PER_SPI_TRANSACTION*2


typedef struct {
    uint8_t startByte[START_STOP_NUM_BYTES];
    uint16_t dataLen;
    s_date_time_t timeData[DATA_LINES_PER_SPI_TRANSACTION];
    uint8_t padding3;
    uint8_t padding4;
    uint8_t gpioData[GPIO_BYTES_PER_SPI_TRANSACTION];
    union {
    	uint8_t adcData[ADC_BYTES_PER_SPI_TRANSACTION];
    	uint16_t adcData_u16[ADC_VALUES_PER_SPI_TRANSACTION];
    };
} spi_msg_1_t;

typedef struct {
    union {
    	uint8_t adcData[ADC_BYTES_PER_SPI_TRANSACTION];
    	uint16_t adcData_u16[ADC_VALUES_PER_SPI_TRANSACTION];
    };
    uint8_t gpioData[GPIO_BYTES_PER_SPI_TRANSACTION];
    uint8_t padding1;
    uint8_t padding2;
    s_date_time_t timeData[DATA_LINES_PER_SPI_TRANSACTION];
    uint16_t dataLen;
    uint8_t stopByte[START_STOP_NUM_BYTES];
} spi_msg_2_t;

typedef struct   __attribute__((aligned(4))) {
    uint8_t msg_no;
	uint16_t dataLen;
    uint8_t padding1[11];
    s_date_time_t timeData[DATA_LINES_PER_SPI_TRANSACTION]; //12*70 = 840
    uint8_t gpioData[GPIO_BYTES_PER_SPI_TRANSACTION]; // 70
    union
    {
        uint8_t adcData[ADC_BYTES_PER_SPI_TRANSACTION]; // 1120
        uint16_t adcData16[ADC_VALUES_PER_SPI_TRANSACTION]; // 560
    };
    // uint16_t crc;
} spi_msg_slow_freq_t;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
