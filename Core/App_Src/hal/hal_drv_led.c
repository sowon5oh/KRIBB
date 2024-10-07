/**
 ******************************************************************************
 * @file           : hal_drv_led.c
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
#include "drv_dac_dac63204.h"
#include "hal_drv_led.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Led_Init(I2C_HandleTypeDef *p_hdl) {
    /* init DAC driver */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    SYS_VERIFY_SUCCESS(DRV_DAC63204_Init(p_hdl));

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Led_Ctrl(HalLedCh_t ch_num, HalLedCtrl_t ctrl_set) {
    SYS_VERIFY_TRUE(ch_num <= HAL_LED_CH_MAX);


    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/

