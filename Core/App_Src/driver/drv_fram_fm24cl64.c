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
#define FM24CL64_I2C_ADDR	0xA0 /* 0b 1 0 1 0  A2 A1 A0 R/FRAM_DATA_MAX_LENW */

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef _fram_write(uint16_t mem_addr, uint8_t *p_data, uint16_t data_len);
HAL_StatusTypeDef _fram_read(uint16_t mem_addr, uint8_t *p_data, uint16_t data_len);
void _fram_w_protection(bool set);

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef *fram_i2c_hdl;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_FM24CL64_Init(I2C_HandleTypeDef *p_handle) {
    SYS_VERIFY_PARAM_NOT_NULL(p_handle);

    /* I2C handle Registration */
    fram_i2c_hdl = (I2C_HandleTypeDef*) p_handle;

    /* Check device ready */
    SYS_VERIFY_SUCCESS(HAL_I2C_IsDeviceReady(fram_i2c_hdl, FM24CL64_I2C_ADDR, 100, HAL_MAX_DELAY));

    /* Write protection */
    _fram_w_protection(true);

    /* Configuration */

    return HAL_OK;
}

HAL_StatusTypeDef DRV_FM24CL64_Write(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);
    SYS_VERIFY_TRUE(mem_addr <= FM24CL64_MAX_ADDRESS);
    SYS_VERIFY_TRUE(mem_addr + data_len <= FM24CL64_MAX_ADDRESS);

    SYS_VERIFY_SUCCESS(_fram_write(mem_addr, p_data, data_len));

    return HAL_OK;
}

HAL_StatusTypeDef DRV_FM24CL64_Read(uint8_t mem_addr, uint16_t data_len, uint8_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);
    SYS_VERIFY_TRUE(mem_addr <= FM24CL64_MAX_ADDRESS);
    SYS_VERIFY_TRUE(mem_addr + data_len <= FM24CL64_MAX_ADDRESS);

    SYS_VERIFY_SUCCESS(_fram_read(mem_addr, p_data, data_len));

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
void _fram_w_protection(bool set) {
    (set == true) ? HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, SET) : HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, RESET);
    HAL_Delay(100);
}

HAL_StatusTypeDef _fram_write(uint16_t mem_addr, uint8_t *p_data, uint16_t data_len) {
    uint8_t write_data[FRAM_DATA_MAX_LEN] = { 0, }; /* Address to read & Data bytes to write */

    SYS_VERIFY_PARAM_NOT_NULL(fram_i2c_hdl);

    write_data[0] = (mem_addr >> 8) & 0xFF;
    write_data[1] = mem_addr & 0xFF;
    memcpy(&write_data[2], p_data, data_len);

    _fram_w_protection(false);
    SYS_VERIFY_SUCCESS(HAL_I2C_Master_Transmit(fram_i2c_hdl, FM24CL64_I2C_ADDR, write_data, data_len + 2, 50));
    _fram_w_protection(true);

    return HAL_OK;
}

HAL_StatusTypeDef _fram_read(uint16_t mem_addr, uint8_t *p_data, uint16_t data_len) {
    uint8_t write_data[2] = { 0, }; /* Address to read */
    uint8_t read_data[FRAM_DATA_MAX_LEN] = { 0, };

    SYS_VERIFY_PARAM_NOT_NULL(fram_i2c_hdl);

    write_data[0] = (mem_addr >> 8) & 0xFF;
    write_data[1] = mem_addr & 0xFF;

    _fram_w_protection(false);
    SYS_VERIFY_SUCCESS(HAL_I2C_Master_Transmit(fram_i2c_hdl, FM24CL64_I2C_ADDR, &write_data[0], 2, 50));
    _fram_w_protection(true);

    SYS_VERIFY_SUCCESS(HAL_I2C_Master_Receive(fram_i2c_hdl, FM24CL64_I2C_ADDR, &read_data[0], data_len, 50));


    memcpy(p_data, &read_data[0], data_len);

    return HAL_OK;
}
