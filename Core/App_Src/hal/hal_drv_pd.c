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
HAL_StatusTypeDef Hal_Pd_Init(SPI_HandleTypeDef *p_hdl, HalPdMeasRespCb_t cb_fn) {
    /* init DAC driver */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    SYS_VERIFY_PARAM_NOT_NULL(cb_fn);
    SYS_VERIFY_SUCCESS(DRV_ADS130B04_Init(p_hdl, cb_fn));

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Pd_Start(void) {
    /* Start ADC */
    SYS_VERIFY_SUCCESS(DRV_ADS130B04_Start());

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Pd_Stop(void) {
    /* Stop ADC */
    SYS_VERIFY_SUCCESS(DRV_ADS130B04_Stop());

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Pd_GetMonitorData(HalPdCh_t ch, int16_t *p_data) {
    SYS_VERIFY_TRUE(ch <= HAL_PD_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    if (HAL_PD_CH_ALL == ch) {
        int16_t pd_data_all[HAL_PD_CH_MAX];
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_3_0, &pd_data_all[HAL_PD_CH_1]));
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_3_1, &pd_data_all[HAL_PD_CH_2]));
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_3_2, &pd_data_all[HAL_PD_CH_3]));

        memcpy(p_data, pd_data_all, sizeof(pd_data_all));
    }
    else {
        int16_t pd_data;
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_3 + ch, &pd_data));

        *p_data = pd_data;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Pd_GetRecvData(HalPdCh_t ch, int16_t *p_data) {
    SYS_VERIFY_TRUE(ch <= HAL_PD_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    if (HAL_PD_CH_ALL == ch) {
        int16_t pd_data_all[HAL_PD_CH_MAX];
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_0, &pd_data_all[HAL_PD_CH_1]));
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_1, &pd_data_all[HAL_PD_CH_2]));
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_2, &pd_data_all[HAL_PD_CH_3]));

        memcpy(p_data, pd_data_all, sizeof(pd_data_all));
    }
    else {
        int16_t pd_data;
        SYS_VERIFY_SUCCESS(DRV_ADS130B04_GetData(DRV_ADS130B04_CH_0 + ch, &pd_data));

        *p_data = pd_data;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Pd_Read(void) {
    SYS_VERIFY_SUCCESS(DRV_ADS130B04_Read());

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
