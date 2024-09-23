/**
 ******************************************************************************
 * @file           : dac_dac63204.c
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
#include "dac_dac63204.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	I2C_HandleTypeDef *i2c_handle;

} dacContext_t;

/* Private define ------------------------------------------------------------*/
#define DAC63204_ADDRESS  				0x90  // 0x92

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
HAL_StatusTypeDef _dacReset(void);
HAL_StatusTypeDef _dacWrite(uint8_t reg, uint16_t data);
HAL_StatusTypeDef _dacRead(uint8_t reg, uint8_t *p_data);

/* Private variables ---------------------------------------------------------*/
dacContext_t dac_context;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DAC_Init(I2C_HandleTypeDef *p_handle) {
	HAL_StatusTypeDef ret = HAL_OK;

	if (p_handle != NULL) {
		/* Regist handle */
		dac_context.i2c_handle = (I2C_HandleTypeDef*) p_handle;

		ret = HAL_I2C_IsDeviceReady(dac_context.i2c_handle, DAC63204_ADDRESS,
				64, HAL_MAX_DELAY);
	} else {
		ret = HAL_ERROR;
	}

	if (ret != HAL_ERROR) {
		ret = _dacReset();

		/* Chip Setting */
		/* Power-up
		 * voltage output on all channels
		 * enables internal reference */
		ret = _dacWrite(DAC_REG_ADDR_COMMON_CFG, 0x1249);

		/* Set channel 0-3 gain setting
		 * 1.5x internal reference (1.8 V) */
		ret = _dacWrite(DAC_REG_ADDR_DAC_0_VOUT_CMP_CFG, 0x0800); //0x0800 = 2048
		ret = _dacWrite(DAC_REG_ADDR_DAC_1_VOUT_CMP_CFG, 0x0800);
		ret = _dacWrite(DAC_REG_ADDR_DAC_2_VOUT_CMP_CFG, 0x0800);
		ret = _dacWrite(DAC_REG_ADDR_DAC_3_VOUT_CMP_CFG, 0x0800);

		/* Configure GPI
		 * for Margin-High
		 * Low trigger for all channels */
		ret = _dacWrite(DAC_REG_ADDR_GPIO_CFG, 0x01F5);

		/* Set Channel 0 */
		/* Slew rate and Code step
		 * CODE_STEP: 2 LSB
		 * SLEW_RATE: 60.72 µs/step */
		ret = _dacWrite(DAC_REG_ADDR_DAC_0_FUNC_CFG, 0x0017);
		/* DAC margin high code
		 * OUTPUT RANGE: 1.8 V
		 * MIRGIN_HIGE: 1.164 V
		 * the 10-bit hex code for 1.164 V is 0x296 With 16-bit left alignment this becomes 0xA540 */
		ret = _dacWrite(DAC_REG_ADDR_DAC_0_MARGIN_HIGH, 0xA540);
		/* DAC margin low code
		 * OUTPUT RANGE: 1.8 V
		 * MIRGIN_LOW: 36 mV
		 * the 10-bit hex code for 36 mV is 0x14. With 16-bit left alignment, this becomes 0x0500 */
		ret = _dacWrite(DAC_REG_ADDR_DAC_0_MARGIN_LOW, 0x0500);

		/* Set Channel 1 */
		ret = _dacWrite(DAC_REG_ADDR_DAC_1_FUNC_CFG, 0x0017);
		ret = _dacWrite(DAC_REG_ADDR_DAC_1_MARGIN_HIGH, 0xA540);
		ret = _dacWrite(DAC_REG_ADDR_DAC_1_MARGIN_LOW, 0x0500);

		/* Set Channel 2 */
		ret = _dacWrite(DAC_REG_ADDR_DAC_2_FUNC_CFG, 0x0017);
		ret = _dacWrite(DAC_REG_ADDR_DAC_2_MARGIN_HIGH, 0xA540);
		ret = _dacWrite(DAC_REG_ADDR_DAC_2_MARGIN_LOW, 0x0500);

		/* Set Channel 3 */
		ret = _dacWrite(DAC_REG_ADDR_DAC_3_FUNC_CFG, 0x0017);
		ret = _dacWrite(DAC_REG_ADDR_DAC_3_MARGIN_HIGH, 0xA540);
		ret = _dacWrite(DAC_REG_ADDR_DAC_3_MARGIN_LOW, 0x0500);

		/* Init Interface */
		ret = _dacWrite(DAC_REG_ADDR_INTERFACE_CFG, 0x0000);

		/* Save to NVM */
		ret = _dacWrite(DAC_REG_ADDR_COMMON_TRIGGER, 0x0002);
	}

	return ret;
}

HAL_StatusTypeDef DAC_SetData(DAC_chSel_t channel, uint16_t data) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint16_t write_data = 0;

	if (data > 0xFF0) //FF0 = 4080
			{
		write_data = 0xFF0;
		LogWarn("Invalid DAC Set value, Value changed. %04X ==> %04X", data,
				write_data);
	} else {
		write_data = (data << 4) & 0xFFF0; //FFF0 = 65520
	}

	if (dac_context.i2c_handle != NULL) {
		switch (channel) {
		case DAC_CH_0:
			ret = _dacWrite(DAC_REG_ADDR_DAC_0_DATA, write_data);
			break;

		case DAC_CH_1:
			ret = _dacWrite(DAC_REG_ADDR_DAC_1_DATA, write_data);
			break;

		case DAC_CH_2:
			ret = _dacWrite(DAC_REG_ADDR_DAC_2_DATA, write_data);
			break;

		case DAC_CH_3:
			ret = _dacWrite(DAC_REG_ADDR_DAC_3_DATA, write_data);
			break;

		case DAC_CH_ALL:
			ret = _dacWrite(DAC_REG_ADDR_DAC_0_DATA, write_data);
			ret = _dacWrite(DAC_REG_ADDR_DAC_1_DATA, write_data);
			ret = _dacWrite(DAC_REG_ADDR_DAC_2_DATA, write_data);
			ret = _dacWrite(DAC_REG_ADDR_DAC_3_DATA, write_data);
			break;

		default:
			ret = HAL_ERROR;
			break;
		}

		if (ret == HAL_OK) {
			LogDebug("DAC Set ch: %d, data: %04X\r\n", channel, data);
		} else {
			LogError("DAC Set failed\r\n");
		}
	} else {
		ret = HAL_ERROR;
	}

	return ret;
}

HAL_StatusTypeDef DAC_CheckStatus(uint8_t *p_read_data) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t read_buff[2] = { 0, };

	if ((dac_context.i2c_handle != NULL) && (p_read_data != NULL)) {
		ret = _dacRead(DAC_REG_ADDR_GENERAL_STATUS, &read_buff[0]);
	} else {
		ret = HAL_ERROR;
	}

	if (ret == HAL_OK) {
		;
	}

	return ret;
}

/* Private user code ---------------------------------------------------------*/
HAL_StatusTypeDef _dacReset(void) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t tx_data[2] = { 0, };

	/* Make packet */
	tx_data[0] = 0x00;
	tx_data[1] = 0x06;

	ret = HAL_I2C_Master_Transmit(dac_context.i2c_handle, DAC63204_ADDRESS,
			&tx_data[0], 2, 50);

	return ret;
}

HAL_StatusTypeDef _dacWrite(uint8_t reg, uint16_t data) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t tx_data[3] = { 0, };

	/* Make packet */
	tx_data[0] = reg;
	tx_data[1] = (data >> 8) & 0xFF;
	tx_data[2] = data & 0xFF;

	ret = HAL_I2C_Master_Transmit(dac_context.i2c_handle, DAC63204_ADDRESS,
			&tx_data[0], 3, 50);

	return ret;
}

HAL_StatusTypeDef _dacRead(uint8_t reg, uint8_t *p_data) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t tx_data = 0x00;
	uint8_t rx_data[2] = { 0, };

	/* Make packet */
	tx_data = reg;

	ret = HAL_I2C_Master_Transmit(dac_context.i2c_handle, DAC63204_ADDRESS,
			&tx_data, 1, 50);

	if (ret == HAL_OK) {
		ret = HAL_I2C_Master_Receive(dac_context.i2c_handle, DAC63204_ADDRESS,
				&rx_data[0], 2, 50);
	}

	if (ret == HAL_OK) {
		memcpy(p_data, &rx_data[0], 2);
	}

	return ret;
}
