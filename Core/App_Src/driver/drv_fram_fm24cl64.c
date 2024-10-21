/**
 ******************************************************************************
 * @file           : drv_fram_fm24cl64.c
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

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FM24CL64_I2C_ADDR	0xA0

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef _fram_write(uint16_t mem_addr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef _fram_read(uint16_t mem_addr, uint8_t *data, uint16_t size);
void _fram_w_protection(bool set);

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef *fram_i2c_hdl;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_FM24CL64_Init(I2C_HandleTypeDef *p_handle) {
    SYS_VERIFY_PARAM_NOT_NULL(p_handle);

    /* I2C handle Registration */
    fram_i2c_hdl = (I2C_HandleTypeDef*) p_handle;

    /* Check device ready */
    SYS_VERIFY_SUCCESS(HAL_I2C_IsDeviceReady(fram_i2c_hdl, FM24CL64_I2C_ADDR, 64, HAL_MAX_DELAY));

    /* Write protection */
    _fram_w_protection(true);

    /* Configuration */

    return HAL_OK;
}

HAL_StatusTypeDef DRV_FM24CL64_Write(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);
    SYS_VERIFY_TRUE(mem_addr <= FM24CL64_MAX_ADDRESS);
    SYS_VERIFY_TRUE(mem_addr + data_len <= FM24CL64_MAX_ADDRESS);

    _fram_w_protection(false);
    SYS_VERIFY_SUCCESS(_fram_write(mem_addr, p_data, data_len));
    _fram_w_protection(true);

    return HAL_OK;
}

HAL_StatusTypeDef DRV_FM24CL64_Read(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);
    SYS_VERIFY_TRUE(mem_addr <= FM24CL64_MAX_ADDRESS);
    SYS_VERIFY_TRUE(mem_addr + data_len <= FM24CL64_MAX_ADDRESS);

    _fram_w_protection(false);
    SYS_VERIFY_SUCCESS(_fram_read(mem_addr, p_data, data_len));

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
void _fram_w_protection(bool set) {
    (set == true) ? HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, SET) : HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, RESET);
    HAL_Delay(1000);
}

HAL_StatusTypeDef _fram_write(uint16_t mem_addr, uint8_t *data, uint16_t size) {
    SYS_VERIFY_PARAM_NOT_NULL(fram_i2c_hdl);

    return HAL_I2C_Mem_Write(fram_i2c_hdl, FM24CL64_I2C_ADDR, mem_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef _fram_read(uint16_t mem_addr, uint8_t *data, uint16_t size) {
    SYS_VERIFY_PARAM_NOT_NULL(fram_i2c_hdl);

    return HAL_I2C_Mem_Read(fram_i2c_hdl, FM24CL64_I2C_ADDR, mem_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}
