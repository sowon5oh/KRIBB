/**
 ******************************************************************************
 * @file           : hal_drv_temperature.c
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
#include "hal_drv_temperature.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FEATURE_HAL_TEMPERATURE_TEST 0

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static HalTempData_t temp_data;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Temp_Init(ADC_HandleTypeDef *p_hdl) {
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    SYS_VERIFY_SUCCESS(DRV_LMT86LP_Init(p_hdl));

#if FEATURE_HAL_TEMPERATURE_TEST
    SYS_LOG_INFO("[Test] Temperature ADC");
    SYS_VERIFY_SUCCESS(DRV_LMT86LP_Start());
#endif
    
    return HAL_OK;
}

void Hal_Temp_AdcCb(void) {
    SYS_VERIFY_SUCCESS_VOID(DRV_LMT86LP_GetValue(&temp_data));
    SYS_LOG_INFO("Temperature ADC Result: %d, %d, %d", temp_data.adc[HAL_TEMP_CH_0], temp_data.adc[HAL_TEMP_CH_1], temp_data.adc[HAL_TEMP_CH_2]);
}

HAL_StatusTypeDef Hal_Temp_Start(void) {
    SYS_VERIFY_SUCCESS(DRV_LMT86LP_Start());
    return HAL_OK;
}

HAL_StatusTypeDef Hal_Temp_Stop(void) {
    SYS_VERIFY_SUCCESS(DRV_LMT86LP_Stop());
    return HAL_OK;
}

HAL_StatusTypeDef Hal_Temp_GetData(HalTempData_t *p_data) {
    memcpy(p_data, &temp_data, sizeof(HalTempData_t));

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/

