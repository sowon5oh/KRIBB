/**
 ******************************************************************************
 * @file           : user_uart.c
 * @brief          :
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "user_uart.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	UART_HandleTypeDef *uart_handle;
} uartContext_t;

/* Private define ------------------------------------------------------------*/
#define TX_MSG_MAX_LEN	64
#define TX_BUFFER_SIZE 	(TX_MSG_MAX_LEN + 1)
#define RX_MSG_MAX_LEN	64
#define RX_BUFFER_SIZE 	(RX_MSG_MAX_LEN + 1)
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef _uartSendData(uint8_t *p_data, uint16_t len);
HAL_StatusTypeDef _uartReceiveData(uint8_t *p_data, uint16_t len);

/* Private variables ---------------------------------------------------------*/
uartContext_t uart_context;
uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef UART_Init(UART_HandleTypeDef *p_handle) {
	HAL_StatusTypeDef ret = HAL_OK;

	if (p_handle != NULL) {
		/* Regist handle */
		uart_context.uart_handle = (UART_HandleTypeDef*) p_handle;

		/* Read Start */
		HAL_UART_Receive_IT(uart_context.uart_handle, rx_buffer, 1);
	} else {
		ret = HAL_ERROR;
	}

	return ret;
}

HAL_StatusTypeDef UART_SendMMI(uint8_t *p_data, uint16_t len) {
	HAL_StatusTypeDef ret = HAL_OK;

	if (p_data == NULL) {
		ret = HAL_ERROR;
        SYS_LOG_ERR("Invalid parameter");
	} else {
		ret = _uartSendData(p_data, len);
	}

	return ret;
}

/* Private user code ---------------------------------------------------------*/
HAL_StatusTypeDef _uartSendData(uint8_t *p_data, uint16_t len) {
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_UART_Transmit_IT(uart_context.uart_handle, p_data, len);

	return ret;
}

HAL_StatusTypeDef _uartReceiveData(uint8_t *p_data, uint16_t len) {
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_UART_Receive_IT(uart_context.uart_handle, p_data, len);

	return ret;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == uart_context.uart_handle->Instance) {
        SYS_LOG_INFO("Send Done");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    uint8_t ch;
	if (huart->Instance == uart_context.uart_handle->Instance) {
//		ch = rx_buffer[0];
//		MMI_Decoder(&ch, 1); //TODO

		HAL_UART_Receive_IT(uart_context.uart_handle, rx_buffer, 1);
	}
}
