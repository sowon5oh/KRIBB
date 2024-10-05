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
DrvLmt86lpCb_t tempsensor_adc_cb;
/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_LMT86LP_Init(ADC_HandleTypeDef *p_hdl, DrvLmt86lpCb_t cb_fn) {
    /* ADC handle registration */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    tempsensor_adc_hdl = p_hdl;

    /* ADC read callback function registration */
    SYS_VERIFY_PARAM_NOT_NULL(cb_fn);
    tempsensor_adc_cb = cb_fn;

    return HAL_OK;
}

HAL_StatusTypeDef DRV_LMT86LP_Start(void) {
    return HAL_ADC_Start_IT(tempsensor_adc_hdl);
}

HAL_StatusTypeDef DRV_LMT86LP_Stop(void) {
    return HAL_ADC_Stop_IT(tempsensor_adc_hdl);
}

HAL_StatusTypeDef DRV_LMT86LP_GetValue(void) {
    uint32_t adc_val[3];

    adc_val[0] = HAL_ADC_GetValue(tempsensor_adc_hdl);
    adc_val[1] = HAL_ADC_GetValue(tempsensor_adc_hdl);
    adc_val[2] = HAL_ADC_GetValue(tempsensor_adc_hdl);

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/

