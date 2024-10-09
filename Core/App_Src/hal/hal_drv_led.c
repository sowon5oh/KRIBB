/**
 ******************************************************************************
 * @file           : hal_drv_led.c
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
#include "drv_dac_dac63204.h"
#include "hal_drv_led.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FEATURE_HAL_LED_TEST 0

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void _led_ctrl(HalLedCh_t ch_num, uint16_t set_data);

/* Private variables ---------------------------------------------------------*/

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef Hal_Led_Init(I2C_HandleTypeDef *p_hdl) {
    /* init DAC driver */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    SYS_VERIFY_SUCCESS(DRV_DAC63204_Init(p_hdl));

#if FEATURE_HAL_LED_TEST
    SYS_LOG_TEST("LED 1 TEST");
    uint8_t test_data;
    DRV_DAC63204_CheckStatus(&test_data);

    _led_ctrl(HAL_LED_CH_1, 0xA000);
    _led_ctrl(HAL_LED_CH_2, 0xA000);
    _led_ctrl(HAL_LED_CH_3, 0xA000);
    HAL_Delay(5000);
    _led_ctrl(HAL_LED_CH_1, 0x0000);
    _led_ctrl(HAL_LED_CH_2, 0x0000);
    _led_ctrl(HAL_LED_CH_3, 0x0000);
#endif

    return HAL_OK;
}

HAL_StatusTypeDef Hal_Led_Ctrl(HalLedCh_t ch_num, uint16_t set_data) {
    SYS_VERIFY_TRUE(ch_num <= HAL_LED_CH_NUM);

    _led_ctrl(ch_num, set_data);

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
static void _led_ctrl(HalLedCh_t ch_num, uint16_t set_data) {
    switch (ch_num) {
        case HAL_LED_CH_1:
            DRV_DAC63204_SetData(DRV_DAC63204_CH_0, set_data);
            break;

        case HAL_LED_CH_2:
            DRV_DAC63204_SetData(DRV_DAC63204_CH_1, set_data);
            break;

        case HAL_LED_CH_3:
            DRV_DAC63204_SetData(DRV_DAC63204_CH_2, set_data);
            break;

        default:
            SYS_LOG_ERR("Invalid LED Ch");
            break;
    }
}
