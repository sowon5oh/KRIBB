/**
 ******************************************************************************
 * @file           : drv_tempsensor_lmt86lp.h
 * @brief          : Header file for drv_tempsensor_lmt86lp.c.
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
#ifndef _DRV_LMT86LP_H_
#define _DRV_LMT86LP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal_drv_temperature.h"

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef DRV_LMT86LP_Init(ADC_HandleTypeDef *p_hdl);
HAL_StatusTypeDef DRV_LMT86LP_Start(void);
HAL_StatusTypeDef DRV_LMT86LP_Stop(void);
HAL_StatusTypeDef DRV_LMT86LP_SaveValue(void);
HAL_StatusTypeDef DRV_LMT86LP_GetValue(HalTempData_t *p_data);

#ifdef __cplusplus
}
#endif

#endif /* _DRV_LMT86LP_H_ */
