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
#include "hal_drv_heater.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static void _temp_read_cb(uint32_t *p_data);

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Temp_Init(ADC_HandleTypeDef *p_hdl) {
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);

    DRV_LMT86LP_Init(p_hdl, _temp_read_cb);

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Temp_Read(void) {
    uint32_t adc_val_1;

    DRV_LMT86LP_GetValue();

}

/* Private user code ---------------------------------------------------------*/

static void _temp_read_cb(uint32_t *p_data) {

}
