/**
 ******************************************************************************
 * @file           : hal_drv_led_driver.h
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
    HAL_LED_CH_NUM_1 = 0,
    HAL_LED_CH_NUM_2,
    HAL_LED_CH_NUM_3,
    HAL_LED_CH_NUM_ALL,
    HAL_LED_CH_NUM_MAX
} HalLedChNUm_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Hal_Led_Init(void);
HAL_StatusTypeDef Hal_Led_Ctrl(HalLedChNUm_t ch_num);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DRV_LED_DRIVER_H */
