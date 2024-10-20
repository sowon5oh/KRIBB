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
#define TX_MSG_MAX_LEN	MMI_PROTOCOL_TX_MSG_LEN_MAX
#define TX_BUFFER_SIZE 	(TX_MSG_MAX_LEN * 2)
#define RX_MSG_MAX_LEN	MMI_PROTOCOL_RX_MSG_LEN_MAX
#define RX_BUFFER_SIZE 	(RX_MSG_MAX_LEN * 2)
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef _uartSendData(uint8_t *p_data, uint16_t len);
static void _hex_to_string(uint8_t *hex_array, size_t hex_array_len, char *output_string);

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *uart_hdl;
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static char tx_str_buffer[(TX_BUFFER_SIZE * 2) + 1] = {
    '\0', };

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

#if (MMI_MSG_DEBUG_LOG == 1)
    _hex_to_string(tx_buffer, len, tx_str_buffer);
#endif

    return HAL_OK;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == uart_hdl->Instance) {
#if (MMI_MSG_DEBUG_LOG == 1)
        SYS_LOG_DEBUG("Uart Message Send Done: %s", tx_str_buffer);
        memset(tx_str_buffer, 0, sizeof(tx_str_buffer));
#endif
        memset(tx_buffer, 0, sizeof(tx_buffer));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == uart_hdl->Instance) {
        Task_MMI_Decoder(&rx_buffer[0], 1);
        HAL_UART_Receive_IT(uart_hdl, rx_buffer, 1);
    }
}

/* Private user code ---------------------------------------------------------*/
static HAL_StatusTypeDef _uartSendData(uint8_t *p_data, uint16_t len) {
    return HAL_UART_Transmit_IT(uart_hdl, p_data, len);
}

static void _hex_to_string(uint8_t *hex_array, size_t hex_array_len, char *output_string) {
    for (size_t i = 0; i < hex_array_len; i++) {
        // 각 바이트를 16진수 형식으로 변환하여 출력
        sprintf(output_string + (i * 2), "%02X", hex_array[i]);
    }
    output_string[hex_array_len * 2] = '\0'; // Null-terminator 추가
}
