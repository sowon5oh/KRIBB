/**
 ******************************************************************************
 * @file           : drv_tempsensor_lmt86lp.c
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
#include "drv_tempsensor_lmt86lp.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef *tempsensor_adc_hdl;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef TEMPSENSOR_Init(ADC_HandleTypeDef *p_handle) {
    SYS_VERIFY_PARAM_NOT_NULL(p_handle);

    /* ADC handle Registration */
    tempsensor_adc_hdl = (ADC_HandleTypeDef*) p_handle;

    return HAL_OK;
}

HAL_StatusTypeDef TEMPSENSOR_Start() {
    return HAL_ADC_Start_IT(tempsensor_adc_hdl);
}

HAL_StatusTypeDef TEMPSENSOR_Stop() {
    return HAL_ADC_Stop_IT(tempsensor_adc_hdl);
}

HAL_StatusTypeDef TEMPSENSOR_Read() {
    //TODO
    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/

