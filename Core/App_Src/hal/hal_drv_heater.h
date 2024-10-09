/**
 ******************************************************************************
 * @file           : #include hal_drv_heater.h
 * @brief          : Header file for hal_drv_heater.c.
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
#ifndef _HAL_DRV_HEATER_DRIVER_H
#define _HAL_DRV_HEATER_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum {
    HAL_HEATER_CH_NUM_1 = 0,
    HAL_HEATER_CH_NUM_2,
    HAL_HEATER_CH_NUM_3,
    HAL_HEATER_CH_NUM_MAX,
} HalHeaterCh_t;

typedef enum {
    HAL_HEATER_OFF = 0,
    HAL_HEATER_ON,
} HalHeaterCtrl_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Hal_Heater_Init(void);
HAL_StatusTypeDef Hal_Heater_Ctrl(HalHeaterCh_t ch_num, HalHeaterCtrl_t ctrl);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DRV_HEATER_DRIVER_H */
