/**
 ******************************************************************************
 * @file           : drv_fram_fm24cl64.h
 * @brief          : Header file for fram_fm24cl64.c.
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
#ifndef _DRV_FRAM_FM24CL64_H
#define _DRV_FRAM_FM24CL64_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define FM24CL64_MAX_ADDRESS 0x1FFF

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef DRV_FM24CL64_Init(I2C_HandleTypeDef *p_handle);
HAL_StatusTypeDef DRV_FM24CL64_Write(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data);
HAL_StatusTypeDef DRV_FM24CL64_Read(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data);

#ifdef __cplusplus
}
#endif

#endif /* _DRV_FRAM_FM24CL64_H */
