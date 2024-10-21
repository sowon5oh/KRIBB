/**
 ******************************************************************************
 * @file           : hal_drv_temperature.c
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
#include "drv_fram_fm24cl64.h"
#include "hal_drv_fram.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FEATURE_HAL_FRAM_TEST 0

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef _fram_write(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data);
static HAL_StatusTypeDef _fram_read(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data);

/* Private variables ---------------------------------------------------------*/

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Fram_Init(I2C_HandleTypeDef *p_hdl) {
    /* init Fram driver */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    SYS_VERIFY_SUCCESS(DRV_FM24CL64_Init(p_hdl));

#if ( FEATURE_HAL_FRAM_TEST == 1 )
    uint8_t test_data[5] = { 0x01, 0x02, 0x03, 0x04, 0x05 };

    SYS_LOG_INFO("[Test] Fram Write, Addr: 0, Len: 5, Data: 0x0102030405");
    _fram_write(0, 5, test_data);
#endif
    
    return HAL_OK;
}

HAL_StatusTypeDef Hal_Fram_Write(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);
    SYS_VERIFY_TRUE(0 != data_len);

    return _fram_write(mem_addr, data_len, p_data);
}

HAL_StatusTypeDef Hal_Fram_Read(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);
    SYS_VERIFY_TRUE(0 != data_len);

    return _fram_read(mem_addr, data_len, p_data);
}

/* Private user code ---------------------------------------------------------*/
static HAL_StatusTypeDef _fram_write(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    uint8_t temp_data[FRAM_DATA_MAX_LEN];

    SYS_VERIFY_SUCCESS(DRV_FM24CL64_Write(mem_addr, data_len, p_data));

    /* Validation */
    SYS_VERIFY_SUCCESS(DRV_FM24CL64_Read(mem_addr, data_len, temp_data));

    for (uint8_t idx = 0; idx < data_len; idx++) {
        if (temp_data[idx] != p_data[idx]) {
            SYS_LOG_ERR("Fram write failed! Idx: %d / Write: 0x%02X / Written: 0x%02X", idx, p_data[idx], temp_data[idx]);
            return HAL_ERROR;
        }
        else {
            SYS_LOG_DEBUG("Fram write success! Idx: %d, Write: 0x%02X", idx, temp_data[idx]);
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef _fram_read(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    SYS_VERIFY_SUCCESS(DRV_FM24CL64_Read(mem_addr, data_len, p_data));

    return HAL_OK;
}

