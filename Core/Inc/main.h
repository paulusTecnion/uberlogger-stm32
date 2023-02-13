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
#include "msg.h"

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


void Idle_Handler(Message_t * msg);
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
#define DATA_OVERRUN_Pin GPIO_PIN_15
#define DATA_OVERRUN_GPIO_Port GPIOA
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
