/**
 ******************************************************************************
 * @file           : hal_drv_heater_driver.c
 * @brief          :
 * @date           : 2024.09.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 UNIOTECH.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hal_drv_heater.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Heater_Init(void) {
    HAL_GPIO_WritePin(HEAT_CON1_GPIO_Port, HEAT_CON1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HEAT_CON2_GPIO_Port, HEAT_CON2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HEAT_CON3_GPIO_Port, HEAT_CON3_Pin, GPIO_PIN_RESET);

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Heater_Ctrl(HalHeaterChNUm_t ch_num) {
    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/

