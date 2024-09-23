/**
 ******************************************************************************
 * @file           : user_uart.h
 * @brief          : Header file for user_uart.c.
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
#ifndef _USER_UART_H
#define _USER_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef UART_Init(UART_HandleTypeDef *p_handle);
HAL_StatusTypeDef UART_SendMMI(uint8_t *p_data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* _USER_UART_H */
