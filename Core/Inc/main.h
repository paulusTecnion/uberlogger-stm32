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
	MAIN_ADC_START,
	MAIN_ADC_CONVERTING,
	MAIN_SPI_START,
	MAIN_SPI_BUSY,
	MAIN_COMPLETE,
	MAIN_ERROR
};

enum  {
	CMD_NO_CMD = 0x00,
	CMD_ADC_START = 0x01,
	CMD_ADC_EXIT,
	CMD_CONFIG_START,
	CMD_CONFIG_EXIT,
	CMD_CONFIG_SET_SFREQ,
	CMD_CONFIG_SET_ADC_BITS,
	CMD_CONFIG_SET_DAC_PWM
};



enum   {
	RESP_OK = 0x01,
	RESP_NOK
};
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
void Config_Handler(uint8_t* aTxBuffer, uint8_t* aRxBuffer);
void Idle_Handler(uint8_t* aTxBuffer, uint8_t* aRxBuffer);

uint8_t Config_Set_Sample_freq(uint8_t sampleFreq);

uint8_t Send_OK(void);
uint8_t Send_NOK(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN_RANGE_SELECT_CLK_Pin GPIO_PIN_15
#define AIN_RANGE_SELECT_CLK_GPIO_Port GPIOA
#define AIN_RANGE_SELECT_CLR_Pin GPIO_PIN_0
#define AIN_RANGE_SELECT_CLR_GPIO_Port GPIOD
#define AIN_PULLUP_SELECT_CLK_Pin GPIO_PIN_1
#define AIN_PULLUP_SELECT_CLK_GPIO_Port GPIOD
#define AIN_PULLUP_SELECT_CLR_Pin GPIO_PIN_2
#define AIN_PULLUP_SELECT_CLR_GPIO_Port GPIOD
#define DATA_RDY_Pin GPIO_PIN_4
#define DATA_RDY_GPIO_Port GPIOB
#define ADC_EN_Pin GPIO_PIN_5
#define ADC_EN_GPIO_Port GPIOB
#define DATA_RDY_DUP_Pin GPIO_PIN_6
#define DATA_RDY_DUP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
