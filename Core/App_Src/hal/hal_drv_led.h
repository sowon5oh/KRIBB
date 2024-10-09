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
    HAL_LED_CH_NUM,
} HalLedCh_t;

#define HAL_LED_SET_OFF 0x0000
/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Hal_Led_Init(I2C_HandleTypeDef *p_hdl);
HAL_StatusTypeDef Hal_Led_Ctrl(HalLedCh_t ch_num, uint16_t set_data);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DRV_LED_DRIVER_H */
