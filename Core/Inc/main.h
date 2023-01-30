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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum  {
	MAIN_IDLE = 0x01,
	MAIN_CONFIG,
	MAIN_LOGGING,
	MAIN_ERROR
};

typedef enum  {
	CMD_NOP = 0x00,
	CMD_SETTINGS_MODE = 0x01,
	CMD_MEASURE_MODE,
	CMD_SET_RESOLUTION,
	CMD_SET_SAMPLE_RATE,
	CMD_SET_ADC_CHANNELS_ENABLED,
	CMD_UNKNOWN
} spi_cmd_esp_t;



enum   {
	RESP_OK = 0x01,
	RESP_NOK
};

typedef enum {
	CMD_RESP_NOP = 0x00,
	CMD_RESP_OK,
	CMD_RESP_NOK
} spi_cmd_resp_t;

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
void Config_Handler();
void Idle_Handler();


uint8_t Send_OK(void);
uint8_t Send_NOK(void);
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
#define AIN_PULLUP_SELECT_CLR_Pin GPIO_PIN_3
#define AIN_PULLUP_SELECT_CLR_GPIO_Port GPIOD
#define STM_DATA_RDY_Pin GPIO_PIN_6
#define STM_DATA_RDY_GPIO_Port GPIOB
#define STM_ADC_EN_Pin GPIO_PIN_7
#define STM_ADC_EN_GPIO_Port GPIOB
#define STM_ADC_EN_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
