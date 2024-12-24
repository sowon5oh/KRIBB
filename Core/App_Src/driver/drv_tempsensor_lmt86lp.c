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
/* Constants for the temperature calculation */
#define TEMPERATURE_SLOPE    -0.0735
#define TEMPERATURE_OFFSET   191.94

/* ADC digit range */
#define ADC_MIN_DIGIT 1518
#define ADC_MAX_DIGIT 2607
#define TEMPERATURE_MAX_VALUE 80.0f
#define TEMPERATURE_MIN_VALUE 0.0f

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static float _digit_to_temperature(uint16_t digit);

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef *tempsensor_adc_hdl;
uint16_t adc_read_data[HAL_TEMP_CH_NUM] = {
    0, };
HalTempData_t temperature_data;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_LMT86LP_Init(ADC_HandleTypeDef *p_hdl) {
    /* ADC handle registration */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    tempsensor_adc_hdl = p_hdl;

    return HAL_OK;
}

HAL_StatusTypeDef DRV_LMT86LP_Start(void) {
#if(CONFIG_FEATURE_TEMPERATURE_DMA_MODE == 1)
    return HAL_ADC_Start_DMA(tempsensor_adc_hdl, (uint32_t*) adc_read_data, 3);
#else
    return HAL_ADC_Start_IT(tempsensor_adc_hdl);
#endif
}

HAL_StatusTypeDef DRV_LMT86LP_Stop(void) {
#if(CONFIG_FEATURE_TEMPERATURE_DMA_MODE == 1)
    return HAL_ADC_Stop_DMA(tempsensor_adc_hdl);
#else
    return HAL_ADC_Stop_IT(tempsensor_adc_hdl);
#endif
}

HAL_StatusTypeDef DRV_LMT86LP_GetValue(HalTempData_t *p_data) {
    static uint16_t adc_val[HAL_TEMP_CH_NUM];

    SYS_VERIFY_PARAM_NOT_NULL(p_data);

#if(CONFIG_FEATURE_TEMPERATURE_DMA_MODE == 1)
    if (adc_read_data[0] > 0) {
        adc_val[HAL_TEMP_CH_0] = adc_read_data[0];
        temperature_data.adc[HAL_TEMP_CH_0] = adc_val[HAL_TEMP_CH_0];
    }
    if (adc_read_data[1] > 0) {
        adc_val[HAL_TEMP_CH_1] = adc_read_data[1];
        temperature_data.adc[HAL_TEMP_CH_1] = adc_val[HAL_TEMP_CH_1];
    }
    if (adc_read_data[2] > 0) {
        adc_val[HAL_TEMP_CH_2] = adc_read_data[2];
        temperature_data.adc[HAL_TEMP_CH_2] = adc_val[HAL_TEMP_CH_2];
    }
#else

        adc_val[HAL_TEMP_CH_0] = (uint16_t) HAL_ADC_GetValue(tempsensor_adc_hdl);
        adc_val[HAL_TEMP_CH_1] = (uint16_t) HAL_ADC_GetValue(tempsensor_adc_hdl);
        adc_val[HAL_TEMP_CH_2] = (uint16_t) HAL_ADC_GetValue(tempsensor_adc_hdl);
#endif

    SYS_LOG_DEBUG("Temperature Adc Raw: %d, %d, %d", temperature_data.adc[HAL_TEMP_CH_0], temperature_data.adc[HAL_TEMP_CH_1], temperature_data.adc[HAL_TEMP_CH_2]);

    temperature_data.degree[HAL_TEMP_CH_0] = _digit_to_temperature(adc_val[0]);
    temperature_data.degree[HAL_TEMP_CH_1] = _digit_to_temperature(adc_val[1]);
    temperature_data.degree[HAL_TEMP_CH_2] = _digit_to_temperature(adc_val[2]);
    SYS_LOG_DEBUG("Temperature Adc Degree: %.2f, %.2f, %.2f", temperature_data.degree[HAL_TEMP_CH_0], temperature_data.degree[HAL_TEMP_CH_1], temperature_data.degree[HAL_TEMP_CH_2]);

    memcpy(p_data, &temperature_data, sizeof(HalTempData_t));

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
/* Function to convert ADC digit to temperature */
static float _digit_to_temperature(uint16_t digit) {
    /* Ensure digit is within the valid range */
    if (digit < ADC_MIN_DIGIT) {
        SYS_LOG_WARN("ADC digit %d is lower than Min digit", digit);
        return TEMPERATURE_MAX_VALUE;
    }
    else if (digit > ADC_MAX_DIGIT) {
        SYS_LOG_WARN("ADC digit %d is higher than Max digit", digit);
        return TEMPERATURE_MIN_VALUE;
    }
    else {
        /* Calculate temperature using the given formula */
        float temperature = TEMPERATURE_SLOPE * digit + TEMPERATURE_OFFSET;
        return temperature;
    }
}

