/**
 ******************************************************************************
 * @file           : hal_drv_pd.c
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
#include "drv_adc_ads130b04.h"
#include "hal_drv_pd.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Pd_Init(SPI_HandleTypeDef *p_hdl) {
    /* init DAC driver */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    SYS_VERIFY_SUCCESS(DRV_ADS130B04_Init(p_hdl));

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Pd_Start(void) {
    return DRV_ADS130B04_Start();
}

HAL_StatusTypeDef Hal_Pd_Stop(void) {
    return DRV_ADS130B04_Stop();
}

HAL_StatusTypeDef Hal_Pd_GetRecvData(HalPdCh_t ch, uint16_t *p_data) {
    //TODO

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Pd_GetMonitorData(HalPdCh_t ch, uint16_t *p_data) {
    //TODO

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
/*
 * MUX Channel Selection Table
 *
 * EN   A1   A0   | Selected Input Connected to Drain (D) Pin
 * ----------------------------------------------------------
 *  0    X    X   | All channels are off
 *  1    0    0   | S1: MP_SIG_CH_1
 *  1    0    1   | S2: MP_SIG_CH_2
 *  1    1    0   | S3: MP_SIG_CH_3
 *  1    1    1   | S4: DAC_CHECK
 *
 * - EN: Enable bit, when set to 0, all channels are off regardless of A1 and A0.
 * - A1, A0: Address bits to select the channel when EN is 1.
 * - X: Don’t care; the value of this bit doesn't matter when EN is 0.
 * - S1, S2, S3, S4: Represents the channels that can be selected.
 */
void _monitor_mux_enable(bool enable) {
    if (enable) {
        HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_RESET);
    }
}

void _monitor_mux_select(HalPdCh_t ch) {
    switch (ch) {
        case HAL_PD_CH_NUM_1:
            HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
            break;
            
        case HAL_PD_CH_NUM_2:
            HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_SET);
            break;
            
        case HAL_PD_CH_NUM_3:
            HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
            break;
            
        default:
            SYS_LOG_ERR("Invalid Monitor Ch Selected: %d", ch);
            return;
    }

    SYS_LOG_DEBUG("Monitor Ch %d Selected", ch);
}
