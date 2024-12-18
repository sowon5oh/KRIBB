/**
 ******************************************************************************
 * @file           : hal_drv_heater.c
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
#define FEATURE_HAL_HEATER_TEST 0

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void _heater_ctrl(HalHeaterCh_t ch_num, HalHeaterCtrl_t ctrl);

/* Private variables ---------------------------------------------------------*/

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Heater_Init(void) {
    _heater_ctrl(HAL_HEATER_CH_1, HAL_HEATER_OFF);
    _heater_ctrl(HAL_HEATER_CH_2, HAL_HEATER_OFF);
    _heater_ctrl(HAL_HEATER_CH_3, HAL_HEATER_OFF);

#if FEATURE_HAL_HEATER_TEST
    SYS_LOG_TEST("HEATER 1 TEST");
    _heater_ctrl(HAL_HEATER_CH_1, HAL_HEATER_ON);
    HAL_Delay(10000);
    _heater_ctrl(HAL_HEATER_CH_1, HAL_HEATER_OFF);
#endif

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Heater_Ctrl(HalHeaterCh_t ch_num, HalHeaterCtrl_t ctrl) {
    SYS_VERIFY_TRUE(ch_num <= HAL_HEATER_CH_ALL);

    if (HAL_HEATER_CH_ALL == ch_num) {
        _heater_ctrl(HAL_HEATER_CH_1, ctrl);
        _heater_ctrl(HAL_HEATER_CH_2, ctrl);
        _heater_ctrl(HAL_HEATER_CH_3, ctrl);
    }
    else {
        _heater_ctrl(ch_num, ctrl);
    }

    SYS_LOG_DEBUG("Heater CH %d %s", ch_num + 1, ctrl? "ON" : "OFF" );

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
static void _heater_ctrl(HalHeaterCh_t ch_num, HalHeaterCtrl_t ctrl) {
    switch (ch_num) {
        case HAL_HEATER_CH_1:
            HAL_GPIO_WritePin(HEAT_CON1_GPIO_Port, HEAT_CON1_Pin, ctrl);
            break;

        case HAL_HEATER_CH_2:
            HAL_GPIO_WritePin(HEAT_CON2_GPIO_Port, HEAT_CON2_Pin, ctrl);
            break;

        case HAL_HEATER_CH_3:
            HAL_GPIO_WritePin(HEAT_CON3_GPIO_Port, HEAT_CON3_Pin, ctrl);
            break;

        default:
            break;
    }
}
