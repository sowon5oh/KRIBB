/**
 ******************************************************************************
 * @file           : drv_dac63204_dac6320463204.c
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

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    I2C_HandleTypeDef *i2c_hdl;

} dac63204Context_t;

/* Private define ------------------------------------------------------------*/
#define DAC63204_ADDRESS  				0x90  /* 1001 0000 */

#define DAC_REG_ADDR_COMMON_CFG			0x1F
#define DAC_REG_ADDR_COMMON_TRIGGER		0x20
#define DAC_REG_ADDR_GENERAL_STATUS		0x22
#define DAC_REG_ADDR_GPIO_CFG			0x24
#define DAC_REG_ADDR_INTERFACE_CFG		0x26
#define DAC_REG_ADDR_DAC_0_VOUT_CMP_CFG	0x03
#define DAC_REG_ADDR_DAC_1_VOUT_CMP_CFG	0x09
#define DAC_REG_ADDR_DAC_2_VOUT_CMP_CFG	0x0F
#define DAC_REG_ADDR_DAC_3_VOUT_CMP_CFG	0x15
#define DAC_REG_ADDR_DAC_0_DATA			0x19  /* CH1 LED V/I Volt */
#define DAC_REG_ADDR_DAC_1_DATA			0x1A  /* CH2 LED V/I Volt */
#define DAC_REG_ADDR_DAC_2_DATA			0x1B  /* CH3 LED V/I Volt */
#define DAC_REG_ADDR_DAC_3_DATA			0x1C  /* DAC_BIT_OUT */
#define DAC_REG_ADDR_DAC_0_FUNC_CFG		0x06
#define DAC_REG_ADDR_DAC_1_FUNC_CFG		0x0C
#define DAC_REG_ADDR_DAC_2_FUNC_CFG		0x12
#define DAC_REG_ADDR_DAC_3_FUNC_CFG		0x18
#define DAC_REG_ADDR_DAC_0_MARGIN_HIGH	0x01
#define DAC_REG_ADDR_DAC_1_MARGIN_HIGH	0x07
#define DAC_REG_ADDR_DAC_2_MARGIN_HIGH 	0x0D
#define DAC_REG_ADDR_DAC_3_MARGIN_HIGH	0x13
#define DAC_REG_ADDR_DAC_0_MARGIN_LOW	0x02
#define DAC_REG_ADDR_DAC_1_MARGIN_LOW	0x08
#define DAC_REG_ADDR_DAC_2_MARGIN_LOW 	0x0E
#define DAC_REG_ADDR_DAC_3_MARGIN_LOW	0x14

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef _dac63204Reset(void);
HAL_StatusTypeDef _dac63204Write(uint8_t reg, uint16_t data);
HAL_StatusTypeDef _dac63204Read(uint8_t reg, uint8_t *p_data);

/* Private variables ---------------------------------------------------------*/
dac63204Context_t dac63204_context;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_DAC63204_Init(I2C_HandleTypeDef *p_hdl) {
    /* attach i2c handle */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    dac63204_context.i2c_hdl = p_hdl;

    /* check i2c communication status */
    SYS_VERIFY_SUCCESS(HAL_I2C_IsDeviceReady(dac63204_context.i2c_hdl, DAC63204_ADDRESS, 64, HAL_MAX_DELAY));

    /* reset */
    SYS_VERIFY_SUCCESS(_dac63204Reset());

    /* Power-up
     * voltage output on all channels
     * enables internal reference */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_COMMON_CFG, 0x1249));

    /* Set channel 0-3 gain setting
     * 1.5x internal reference (1.8 V) */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_0_VOUT_CMP_CFG, 0x0800)); /* 0x0800 = 2048 */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_1_VOUT_CMP_CFG, 0x0800));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_2_VOUT_CMP_CFG, 0x0800));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_3_VOUT_CMP_CFG, 0x0800));

    /* Configure GPI
     * for Margin-High
     * Low trigger for all channels */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_GPIO_CFG, 0x01F5));

    /* Set Channel 0 */
    /* Slew rate and Code step
     * CODE_STEP: 2 LSB
     * SLEW_RATE: 60.72 µs/step */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_0_FUNC_CFG, 0x0017));
    /* DAC margin high code
     * OUTPUT RANGE: 1.8 V
     * MIRGIN_HIGE: 1.164 V
     * the 10-bit hex code for 1.164 V is 0x296 With 16-bit left alignment this becomes 0xA540 */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_0_MARGIN_HIGH, 0xA540));
    /* DAC margin low code
     * OUTPUT RANGE: 1.8 V
     * MIRGIN_LOW: 36 mV
     * the 10-bit hex code for 36 mV is 0x14. With 16-bit left alignment, this becomes 0x0500 */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_0_MARGIN_LOW, 0x0500));

    /* Set Channel 1 */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_1_FUNC_CFG, 0x0017));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_1_MARGIN_HIGH, 0xA540));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_1_MARGIN_LOW, 0x0500));

    /* Set Channel 2 */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_2_FUNC_CFG, 0x0017));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_2_MARGIN_HIGH, 0xA540));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_2_MARGIN_LOW, 0x0500));

    /* Set Channel 3 */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_3_FUNC_CFG, 0x0017));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_3_MARGIN_HIGH, 0xA540));
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_3_MARGIN_LOW, 0x0500));

    /* Init Interface */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_INTERFACE_CFG, 0x0000));

    /* Save to NVM */
    SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_COMMON_TRIGGER, 0x0002));

    return HAL_OK;
}

HAL_StatusTypeDef DRV_DAC63204_SetData(Dac63204_chSel_t ch, uint16_t data) {
    uint16_t write_data = 0;

    SYS_VERIFY_TRUE(DRV_DAC63204_CH_NUM > ch);

    if (data > 0xFFF) {
        write_data = 0xFFF;
        SYS_LOG_WARN("DAC Set value too high, Value changed. %04X ==> 0xFFF", data, write_data);
    }
    else {
        write_data = data;
    }

    switch (ch) {
        case DRV_DAC63204_CH_0:
            SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_0_DATA, write_data));
            break;

        case DRV_DAC63204_CH_1:
            SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_1_DATA, write_data));
            break;

        case DRV_DAC63204_CH_2:
            SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_2_DATA, write_data));
            break;

        case DRV_DAC63204_CH_3:
            SYS_VERIFY_SUCCESS(_dac63204Write(DAC_REG_ADDR_DAC_3_DATA, write_data));
            break;

        default:
            SYS_LOG_ERR("Invalid DAC Ch");
            return HAL_ERROR;
    }

    SYS_LOG_DEBUG("DAC Set ch: %d, data: %04X", ch, write_data);

    return HAL_OK;
}

HAL_StatusTypeDef DRV_DAC63204_CheckStatus(uint8_t *p_read_data) {
    uint16_t read_buff = 0;

    SYS_VERIFY_PARAM_NOT_NULL(p_read_data);
    SYS_VERIFY_PARAM_NOT_NULL(dac63204_context.i2c_hdl);
    SYS_VERIFY_SUCCESS(_dac63204Read(DAC_REG_ADDR_GENERAL_STATUS, (uint8_t *)&read_buff));

    SYS_LOG_DEBUG("DAC Version ID: %d", BF_GET(read_buff,1,0));
    SYS_LOG_DEBUG("DAC Device ID: %d", BF_GET(read_buff,1,2));
    SYS_LOG_DEBUG("DAC CH0 State: %d", BF_GET(read_buff,1,9));
    SYS_LOG_DEBUG("DAC CH1 State: %d", BF_GET(read_buff,1,10));
    SYS_LOG_DEBUG("DAC CH2 State: %d", BF_GET(read_buff,1,11));

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
HAL_StatusTypeDef _dac63204Reset(void) {
    uint8_t tx_data[2] = { 0, };

    SYS_VERIFY_PARAM_NOT_NULL(dac63204_context.i2c_hdl);

    /* Make packet */
    tx_data[0] = 0x00;
    tx_data[1] = 0x06;

    SYS_VERIFY_SUCCESS(HAL_I2C_Master_Transmit(dac63204_context.i2c_hdl, DAC63204_ADDRESS, &tx_data[0], 2, 50));

    return HAL_OK;
}

HAL_StatusTypeDef _dac63204Write(uint8_t reg, uint16_t data) {
    uint8_t tx_data[3] = { 0, };

    SYS_VERIFY_PARAM_NOT_NULL(dac63204_context.i2c_hdl);

    /* Make packet */
    tx_data[0] = reg;
    tx_data[1] = (data >> 8) & 0xFF;
    tx_data[2] = data & 0xFF;

    SYS_VERIFY_SUCCESS(HAL_I2C_Master_Transmit(dac63204_context.i2c_hdl, DAC63204_ADDRESS, &tx_data[0], 3, 50));

    return HAL_OK;
}

HAL_StatusTypeDef _dac63204Read(uint8_t reg, uint8_t *p_data) {
    uint8_t tx_data = 0x00;
    uint8_t rx_data[2] = { 0, };

    SYS_VERIFY_PARAM_NOT_NULL(dac63204_context.i2c_hdl);
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    /* Make packet */
    tx_data = reg;

    SYS_VERIFY_SUCCESS(HAL_I2C_Master_Transmit(dac63204_context.i2c_hdl, DAC63204_ADDRESS, &tx_data, 1, 50));
    SYS_VERIFY_SUCCESS(HAL_I2C_Master_Receive(dac63204_context.i2c_hdl, DAC63204_ADDRESS, &rx_data[0], 2, 50));

    memcpy(p_data, &rx_data[0], 2);

    return HAL_OK;
}
