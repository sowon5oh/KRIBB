/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "data_type.h"
#include "util_debug.h"
#include "app_util.h"
#include "user_uart.h"
#include "user_mmi.h"
#include "adc_ads130b04.h"
#include "dac_dac63204.h"
#include "led_task.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CH1_LED_CON_Pin GPIO_PIN_0
#define CH1_LED_CON_GPIO_Port GPIOC
#define CH2_LED_CON_Pin GPIO_PIN_1
#define CH2_LED_CON_GPIO_Port GPIOC
#define CH3_LED_CON_Pin GPIO_PIN_2
#define CH3_LED_CON_GPIO_Port GPIOC
#define M_SEL_A0_Pin GPIO_PIN_0
#define M_SEL_A0_GPIO_Port GPIOA
#define M_SEL_A1_Pin GPIO_PIN_1
#define M_SEL_A1_GPIO_Port GPIOA
#define M_SEL_EN_Pin GPIO_PIN_2
#define M_SEL_EN_GPIO_Port GPIOA
#define ADC_CS__Pin GPIO_PIN_4
#define ADC_CS__GPIO_Port GPIOA
#define TEMP_CH1_Pin GPIO_PIN_5
#define TEMP_CH1_GPIO_Port GPIOA
#define TEMP_CH2_Pin GPIO_PIN_6
#define TEMP_CH2_GPIO_Port GPIOA
#define TEMP_CH3_Pin GPIO_PIN_7
#define TEMP_CH3_GPIO_Port GPIOA
#define OP_LED_Pin GPIO_PIN_0
#define OP_LED_GPIO_Port GPIOB
#define EEPROM_WP_Pin GPIO_PIN_1
#define EEPROM_WP_GPIO_Port GPIOB
#define HEAT_CON3_Pin GPIO_PIN_13
#define HEAT_CON3_GPIO_Port GPIOB
#define HEAT_CON2_Pin GPIO_PIN_14
#define HEAT_CON2_GPIO_Port GPIOB
#define HEAT_CON1_Pin GPIO_PIN_15
#define HEAT_CON1_GPIO_Port GPIOB
#define ADC_RST__Pin GPIO_PIN_6
#define ADC_RST__GPIO_Port GPIOC
#define ADC_DRDY__Pin GPIO_PIN_7
#define ADC_DRDY__GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
