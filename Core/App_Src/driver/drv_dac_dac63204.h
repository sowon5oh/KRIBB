/**
 ******************************************************************************
 * @file           : drv_dac_dac63204.h
 * @brief          : Header file for dac_dac63204.c.
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
#ifndef _DRV_DAC_DAC63204_H
#define _DRV_DAC_DAC63204_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum {
    DAC_CH_0 = 0,
    DAC_CH_1,
    DAC_CH_2,
    DAC_CH_3,
    DAC_CH_NUM,
    DAC_CH_ALL,
} DAC_chSel_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

HAL_StatusTypeDef DAC_Init(I2C_HandleTypeDef *p_handle);
HAL_StatusTypeDef DAC_SetData(DAC_chSel_t channel, uint16_t data);
HAL_StatusTypeDef DAC_CheckStatus(uint8_t *p_read_data);

#ifdef __cplusplus
}
#endif

#endif /* _DRV_DAC_DAC63204_H */
