/**
 ******************************************************************************
 * @file           : hal_drv_led.h
 * @brief          : Header file for hal_drv_led_driver.c.
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
#ifndef _HAL_DRV_LED_DRIVER_H
#define _HAL_DRV_LED_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum {
    HAL_LED_CH_1 = 0,
    HAL_LED_CH_2,
    HAL_LED_CH_3,
    HAL_LED_CH_ALL,
    HAL_LED_CH_MAX = HAL_LED_CH_ALL,
} HalLedCh_t;

typedef enum {
    HAL_LED_OFF = 0,
    HAL_LED_ON = 1,
} HalLedCtrl_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Hal_Led_Init(I2C_HandleTypeDef *p_hdl);
HAL_StatusTypeDef Hal_Led_Ctrl(HalLedCh_t ch_num, HalLedCtrl_t ctrl_set);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DRV_LED_DRIVER_H */
