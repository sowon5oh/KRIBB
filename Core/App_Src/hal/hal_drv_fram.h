/**
 ******************************************************************************
 * @file           : #include hal_drv_fram.h
 * @brief          : Header file for hal_drv_fram.c.
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
#ifndef _HAL_DRV_FRAM_H
#define _HAL_DRV_FRAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Hal_Fram_Init(I2C_HandleTypeDef *p_hdl);
HAL_StatusTypeDef Hal_Fram_Write(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data);
HAL_StatusTypeDef Hal_Fram_Read(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DRV_FRAM_H */
