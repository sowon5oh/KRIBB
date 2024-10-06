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
UART_HandleTypeDef *uart_hdl;
uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef UART_Init(UART_HandleTypeDef *p_hdl) {
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);

    /* Regist handle */
    uart_hdl = (UART_HandleTypeDef*) p_hdl;

    /* Read Start */
    HAL_UART_Receive_IT(uart_hdl, rx_buffer, 1);

    return HAL_OK;
}

HAL_StatusTypeDef UART_SendMMI(uint8_t *p_data, uint16_t len) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    memcpy(tx_buffer, p_data, len);
    SYS_VERIFY_SUCCESS(_uartSendData(tx_buffer, len));

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
HAL_StatusTypeDef _uartSendData(uint8_t *p_data, uint16_t len) {
    return HAL_UART_Transmit_IT(uart_hdl, p_data, len);
}

HAL_StatusTypeDef _uartReceiveData(uint8_t *p_data, uint16_t len) {
    return HAL_UART_Receive_IT(uart_hdl, p_data, len);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == uart_hdl->Instance) {
        SYS_LOG_DEBUG("Uart Message Send Done");
        memset(tx_buffer, 0, sizeof(tx_buffer));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == uart_hdl->Instance) {
        Task_MMI_Decoder(&rx_buffer[0], 1);
        HAL_UART_Receive_IT(uart_hdl, rx_buffer, 1);
    }
}
