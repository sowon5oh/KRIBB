/**
 ******************************************************************************
 * @file           : #include hal_drv_temperature.h
 * @brief          : Header file for hal_drv_temperature.c.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_DRV_TEMP_DRIVER_H
#define _HAL_DRV_TEMP_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum {
    HAL_TEMP_CH_0 = 0,
    HAL_TEMP_CH_1,
    HAL_TEMP_CH_2,
    HAL_TEMP_CH_ALL = 3,
    HAL_TEMP_CH_NUM = HAL_TEMP_CH_ALL,
} HalTempCh_t;

typedef struct {
    uint16_t adc[HAL_TEMP_CH_NUM];
    float degree[HAL_TEMP_CH_NUM];
} HalTempData_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Hal_Temp_Init(ADC_HandleTypeDef *p_hdl);
void Hal_Temp_AdcCb(void);
HAL_StatusTypeDef Hal_Temp_Start(void);
HAL_StatusTypeDef Hal_Temp_Stop(void);
HAL_StatusTypeDef Hal_Temp_GetData(HalTempData_t *p_data);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DRV_TEMP_DRIVER_H */
