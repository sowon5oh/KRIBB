/**
 ******************************************************************************
 * @file           : adc_ads130b04.c
 * @brief          : Main program body
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
#include "adc_ads130b04.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	ADC_STATE_STANDBY = 0, ADC_STATE_CONTINOUS, /* ADC data are generated constantly at the rate of fDATA = fMOD / OSR. New data are indicated by a DRDY falling edge at this rate */
	ADC_STATE_GLOBAL_CHOP, /* enabled by setting the GC_EN bit in the GLOBAL_CHOP_CFG register */
} adcStateMode_t;

typedef enum {
	ADC_CLOCK_INTERNAL_OSC = 0, ADC_CLOCK_EXTERNAL_CLOCK,
} adcClockSel_t;

typedef enum {
	/* SPS: Samples per seconds */
	ADC_OSR_MODE_128 = 0, /* 32 	 ksps */
	ADC_OSR_MODE_256, /* 16 	 ksps */
	ADC_OSR_MODE_512, /*  8 	 ksps */
	ADC_OSR_MODE_1024, /*  4	 ksps */
	ADC_OSR_MODE_2048, /*  2 	 ksps */
	ADC_OSR_MODE_4096, /*  1 	 ksps */
	ADC_OSR_MODE_8192, /*  0.5  ksps */
	ADC_OSR_MODE_16384, /*  0.25 ksps */
} adcOsrMode_t;

typedef enum {
	ADC_PWR_MODE_VERY_LOW_POWER = 0,
	ADC_PWR_MODE_LOW_POWER,
	ADC_PWR_MODE_HIGH_RESOLUTION,
} adcPwrMode_t;

typedef enum {
	ADC_SYNC_MODE_OFF = 0, /* Default */
	ADC_SYNC_MODE_ON,
} adcSyncMode_t;

typedef enum {
	ADC_WORD_SIZE_16_BIT = 0, ADC_WORD_SIZE_24_BIT, /* Default */
	ADC_WORD_SIZE_32_BIT,
} adcSpiWordSize_t;

typedef struct {
	adcSpiWordSize_t size_id;
	uint8_t byte_num;
} adcSpiWordCfg_t;

typedef enum {
	ADC_CH_GAIN_MODE_1 = 0, /* +/- 1.2     V */
	ADC_CH_GAIN_MODE_2, /* +/- 600    mV */
	ADC_CH_GAIN_MODE_4, /* +/- 300    mV */
	ADC_CH_GAIN_MODE_8, /* +/- 150    mV */
	ADC_CH_GAIN_MODE_16, /* +/-  75    mV */
	ADC_CH_GAIN_MODE_32, /* +/-  37.5  mV */
	ADC_CH_GAIN_MODE_64, /* +/-  18.75 mV */
	ADC_CH_GAIN_MODE_128, /* +/-  9.375 mV */
} adcChGainMode_t;

typedef enum {
	ADC_CH_INPUT_MODE_CONN = 0,
	ADC_CH_INPUT_MODE_DISCONN,
	ADC_CH_INPUT_MODE_TEST_POSITIVE,
	ADC_CH_INPUT_MODE_TEST_NEGATIVE,
} adcChInputMode_t;

typedef struct {
	bool enable;
	adcChGainMode_t gain_mode;
	adcChInputMode_t input_mode;
} adcChCfg_t;

typedef enum {
	ADC_CMD_ID_NULL = 0,
	ADC_CMD_ID_RESET,
	ADC_CMD_ID_STANDBY,
	ADC_CMD_ID_WAKEUP,
	ADC_CMD_ID_LOCK,
	ADC_CMD_ID_UNLOCK,
} adcCmdId_t;

typedef struct {
	SPI_HandleTypeDef *spi_handle;

	adcStateMode_t state;

	MeasAvrReq_t adc_cfg;

	/* ADS130B04 Settings */
	bool lock;
	bool global_chop;
	adcSyncMode_t sync_mode;
	adcSpiWordCfg_t word_size;

	/* ADS130B04 Clock Config */
	adcClockSel_t clock_sel;
	adcOsrMode_t osr_mode;
	adcPwrMode_t pwr_mode;
	adcChCfg_t ch_cfg[ADC_CH_NUM];

	/* ADC Average Request & Callback */
	bool adc_read;

	//uint8_t adc_ch;
	uint16_t adc_num;
	uint16_t adc_cnt;
	uint16_t adc_wait_time;

	double adc_sum;

	MeasAvrResultCb_t cb_fn;
} adcContext_t;

/* Private define ------------------------------------------------------------*/
#define ADC_SPI_Select()    		    HAL_GPIO_WritePin(ADC_CS__GPIO_Port, ADC_CS__Pin, GPIO_PIN_RESET)
#define ADC_SPI_Deselect()  		    HAL_GPIO_WritePin(ADC_CS__GPIO_Port, ADC_CS__Pin, GPIO_PIN_SET)

#define ADC_CMD_NULL				    0x0000	/*  */
#define ADC_CMD_RESET				    0x0011	/*  */
#define ADC_CMD_STANDBY				    0x0022	/* places the device in a low-power standby mode */
#define ADC_CMD_WAKEUP				    0x0033	/* returns the device to conversion mode from standby mode */
#define ADC_CMD_LOCK				    0x0555	/*  */
#define ADC_CMD_UNLOCK				    0x0655	/*  */

#define ADC_CMD_RESP_RESET			    0xFF54
#define ADC_CMD_RESP_STANDBY		    ADC_CMD_STANDBY
#define ADC_CMD_RESP_WAKEUP			    ADC_CMD_WAKEUP
#define ADC_CMD_RESP_LOCK			    ADC_CMD_LOCK
#define ADC_CMD_RESP_UNLOCK			    ADC_CMD_UNLOCK

#define ADC_CMD_RREG_HEAD			    0xA000	/* used to read the device registers */
#define ADC_CMD_WREG_HEAD			    0x6000
#define ADC_CMD_W_R_REG_ADDR_BOFF	    7
#define ADC_CMD_W_R_REG_ADDR_BCNT	    6
#define ADC_CMD_W_R_DATA_NUM_BOFF	    0
#define ADC_CMD_W_R_DATA_NUM_BCNT	    7
#define ADC_CMD_RREG_MAX_LEN		    10

/* DEVICE SETTINGS AND STATUS INDICATORS (Read-Only Registers) */
#define ADC_REG_ADDR_ID				    0x00
#define ADC_REG_ADDR_STATUS			    0x01
/* GLOBAL SETTINGS ACROSS CHANNELS */
#define ADC_REG_ADDR_MODE				0x02
#define ADC_REG_ADDR_CLOCK				0x03
#define ADC_PWR_MODE_BOFF				0
#define ADC_PWR_MODE_BCNT				2
#define ADC_OSR_MODE_BOFF				2
#define ADC_OSR_MODE_BCNT				3
#define ADC_CLK_SEL_BOFF			    7
#define ADC_CLK_SEL_BCNT			    1
#define ADC_CH0_EN_BOFF				    8
#define ADC_CH0_EN_BCNT				    1
#define ADC_CH1_EN_BOFF				    9
#define ADC_CH1_EN_BCNT				    1
#define ADC_CH2_EN_BOFF				    10
#define ADC_CH2_EN_BCNT				    1
#define ADC_CH3_EN_BOFF				    11
#define ADC_CH3_EN_BCNT				    1
#define ADC_REG_ADDR_GAIN				0x04
#define ADC_GAIN_CFG_CH0_BOFF			0
#define ADC_GAIN_CFG_CH0_BCNT			3
#define ADC_GAIN_CFG_CH1_BOFF			4
#define ADC_GAIN_CFG_CH1_BCNT			3
#define ADC_GAIN_CFG_CH2_BOFF			8
#define ADC_GAIN_CFG_CH2_BCNT			3
#define ADC_GAIN_CFG_CH3_BOFF			12
#define ADC_GAIN_CFG_CH3_BCNT			3
#define ADC_REG_ADDR_GLOBAL_CHOP_CFG	0x06
/* CHANNEL-SPECIFIC SETTINGS */
#define ADC_REG_ADDR_CH0_CFG			0x09 //9
#define ADC_REG_ADDR_CH1_CFG			0x0E //14
#define ADC_REG_ADDR_CH2_CFG			0x13 //19
#define ADC_REG_ADDR_CH3_CFG			0x18 //24
#define ADC_CH_CFG_MUX_BOFF			    1
#define ADC_CH_CFG_MUX_BCNT			    2
/* REGISTER MAP CRC REGISTER (Read-Only Register) */
#define ADC_REG_ADDR_REG_MAP_CRC		0x3E //62

#define ADC_SPI_WORD_BYTE_NUM_16BIT	    2
#define ADC_SPI_WORD_BYTE_NUM_24BIT	    3
#define ADC_SPI_WORD_BYTE_NUM_32BIT	    4

#define ADC_SPI_CMD_BYTE_SIZE	        2       /* 16 bit */

#define ADC_SPI_WORD_MIN_BUFF_SIZE		ADC_SPI_WORD_BYTE_NUM_32BIT * 6  /* Command, Crc */
#define ADC_SPI_WORD_MAX_BUFF_SIZE		ADC_SPI_WORD_MIN_BUFF_SIZE * 5   /* Command, Crc, Data... */
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void _adcDataReset(void);
HAL_StatusTypeDef _adcSendCmd(adcCmdId_t cmd);
HAL_StatusTypeDef _adcGetData(bool first_read);
HAL_StatusTypeDef _adcGetData2(bool first_read);
HAL_StatusTypeDef _adcSetClockCfg(void);
HAL_StatusTypeDef _adcSetGainCfg(void);
HAL_StatusTypeDef _adcSetChMuxCfg(void);

HAL_StatusTypeDef _adcWriteRegister(uint16_t reg, uint16_t data);
HAL_StatusTypeDef _adcReadRegister(uint16_t reg, uint8_t len, uint16_t *p_data);

HAL_StatusTypeDef _adcTransmitReceive(uint16_t *p_tx_data, int16_t *p_rx_data,
		uint8_t len);
HAL_StatusTypeDef _adcTransmit(uint16_t *p_data, uint8_t len);
HAL_StatusTypeDef _adcReceive(int16_t *p_data, uint8_t len);

void _makeWordPacket(uint16_t *p_data, uint8_t len, uint8_t *p_result);
void _parseWordPacket(uint8_t *p_data, uint8_t len, int16_t *p_result);
/* Private variables ---------------------------------------------------------*/
adcContext_t adc_context = { .adc_read = false, .adc_num = 0, .adc_cnt = 0,
		.adc_wait_time = 0, .word_size.byte_num = ADC_SPI_WORD_BYTE_NUM_24BIT };

uint16_t adc_avr_buff[ADC_CH_NUM][ADC_AVR_BUFF_MAX_NUM];

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef ADC_Init(SPI_HandleTypeDef *p_handle) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint16_t read_data = 0;

	if (p_handle != NULL) {
		/* Regist handle */
		adc_context.spi_handle = (SPI_HandleTypeDef*) p_handle;

		/* Pin Reset */
		HAL_GPIO_WritePin(ADC_RST__GPIO_Port, ADC_RST__Pin, GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(ADC_RST__GPIO_Port, ADC_RST__Pin, GPIO_PIN_SET);

		/* General Config */
		adc_context.state = ADC_STATE_STANDBY;
		adc_context.lock = false;
		adc_context.global_chop = true;
		adc_context.sync_mode = ADC_SYNC_MODE_OFF;
		adc_context.word_size.byte_num = ADC_SPI_WORD_BYTE_NUM_24BIT;

		/* Clock Config */
		adc_context.clock_sel = ADC_CLOCK_INTERNAL_OSC;
		adc_context.osr_mode = ADC_OSR_MODE_1024;
		adc_context.pwr_mode = ADC_PWR_MODE_HIGH_RESOLUTION;

		/* Reset */
		ret = _adcSendCmd(ADC_CMD_ID_RESET);
		if (ret == HAL_OK) {
			ret = _adcSendCmd(ADC_CMD_ID_STANDBY);
		}

		/* Read ID */
		ret = _adcReadRegister(ADC_REG_ADDR_ID, 1, &read_data);
		if (ret == HAL_OK) {
			LogInfo("ADC ID: %04X", read_data); //5404 - 21508
		}

		ret = _adcReadRegister(ADC_REG_ADDR_STATUS, 1, &read_data);
		if (ret == HAL_OK) {
			LogInfo("ADC STATUS: %04X", read_data); //0500 - 1280
		}

		ret = _adcReadRegister(ADC_REG_ADDR_MODE, 1, &read_data);
		if (ret == HAL_OK) {
			LogInfo("ADC MODE: %04X", read_data); //0510 - 1296
		}

		ret = _adcReadRegister(ADC_REG_ADDR_CLOCK, 1, &read_data);
		if (ret == HAL_OK) {
			LogInfo("ADC CLOCK: %04X", read_data); //0F8E - 3970
		}

		/* Standby */
		ret = _adcSendCmd(ADC_CMD_ID_STANDBY);

		/* Initialize */
		adc_context.adc_num = 0;
		adc_context.adc_cnt = 0;
		adc_context.adc_sum = 0;

		/* Ch0 Config */
		adc_context.ch_cfg[ADC_CH_0].enable = true;
		adc_context.ch_cfg[ADC_CH_0].input_mode = ADC_CH_INPUT_MODE_CONN;
		adc_context.ch_cfg[ADC_CH_0].gain_mode = ADC_CH_GAIN_MODE_1;

		/* Ch1 Config */
		adc_context.ch_cfg[ADC_CH_1].enable = true;
		adc_context.ch_cfg[ADC_CH_1].input_mode = ADC_CH_INPUT_MODE_CONN;
		adc_context.ch_cfg[ADC_CH_1].gain_mode = ADC_CH_GAIN_MODE_1;

		/* Ch2 Config */
		adc_context.ch_cfg[ADC_CH_2].enable = true;
		adc_context.ch_cfg[ADC_CH_2].input_mode = ADC_CH_INPUT_MODE_CONN;
		adc_context.ch_cfg[ADC_CH_2].gain_mode = ADC_CH_GAIN_MODE_1;

		/* Ch3 Config */
		adc_context.ch_cfg[ADC_CH_3].enable = true;
		adc_context.ch_cfg[ADC_CH_3].input_mode = ADC_CH_INPUT_MODE_CONN;
		adc_context.ch_cfg[ADC_CH_3].gain_mode = ADC_CH_GAIN_MODE_1;

		/* Apply Config */
		_adcSetClockCfg();
		_adcSetGainCfg();
		_adcSetChMuxCfg();
	} else {
		ret = HAL_ERROR;
	}

	return ret;
}

HAL_StatusTypeDef ADC_Start(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	/* Wake Up */
	ret = _adcSendCmd(ADC_CMD_ID_WAKEUP);

	return ret;
}

void ADC_CFG_Change(MeasAvrReq_t *p_req_info) {
	HAL_StatusTypeDef ret = HAL_OK;

	adc_context.adc_cfg.Mux_ch = p_req_info->Mux_ch;
	adc_context.adc_cfg.ch = p_req_info->ch;
	adc_context.osr_mode = p_req_info->sps; //sps
	adc_context.adc_num = p_req_info->adc_num; //samples
	adc_context.adc_wait_time = p_req_info->wait_time;

	//ret = _adcSetClockCfg();

	if (ret != HAL_OK) {
		LogError("ADC Clock Setting failed");
	}
}

HAL_StatusTypeDef ADC_Stop(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	/* Standby */
	ret = _adcSendCmd(ADC_CMD_ID_STANDBY);

	/* Reset measure data */
	_adcDataReset();

	return ret;
}

void ADC_MeasStop(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	/* Reset measure data */
	_adcDataReset();

	/* Disable ADC Ch */
	adc_context.ch_cfg[ADC_CH_0].enable = false;
	adc_context.ch_cfg[ADC_CH_1].enable = false;
	adc_context.ch_cfg[ADC_CH_2].enable = false;
	adc_context.ch_cfg[ADC_CH_3].enable = false;

	ret = _adcSetClockCfg();

	if (ret != HAL_OK) {
		LogError("ADC Clock Setting failed");
	}
}

void ADC_GetSetting(uint8_t *ch, uint8_t *sps, uint16_t *samples,
		uint16_t *wait_time) {
	*ch = adc_context.adc_cfg.ch;
	*sps = adc_context.osr_mode;
	*samples = adc_context.adc_num;
	*wait_time = adc_context.adc_wait_time;
}

HAL_StatusTypeDef ADC_ReqAvr(MeasAvrReq_t *p_req_info, MeasAvrResultCb_t cb_fn) {
	HAL_StatusTypeDef ret = HAL_OK;
	bool clock_recfg = false;

	if ((p_req_info == NULL) || (cb_fn == NULL)) {
		ret = HAL_ERROR;
		LogError("Invalid Parameter");
	} else {
		adc_context.adc_cfg.ch = p_req_info->ch;
		adc_context.adc_num = p_req_info->adc_num;
		adc_context.adc_cnt = 0;
		adc_context.cb_fn = cb_fn;

		if (adc_context.ch_cfg[adc_context.adc_cfg.ch].enable != true) {
			LogInfo("Enable CH %d", adc_context.adc_cfg.ch);
			adc_context.ch_cfg[adc_context.adc_cfg.ch].enable = true;
			clock_recfg = true;
		}

		if (adc_context.osr_mode != (adcOsrMode_t) p_req_info->sps) {
			LogInfo("Change OSR (SPS) %d ==> %d", adc_context.osr_mode,
					p_req_info->sps);
			adc_context.osr_mode = p_req_info->sps;

			clock_recfg = true;
		}

		if (clock_recfg == true) {
			ret = _adcSetClockCfg();
		}

		if (ret == HAL_OK) {
			/* Get first datay to init FIFO for data ready signal */
			LogDebug("ADC Init FIFO");
			_adcGetData(true);
			_adcGetData(true);

			/* Read Start */
			LogDebug("ADC Read Start");
			adc_context.adc_read = true;
		} else {
			LogError("ADC Meas Setting failed");
		}
	}

	return ret;
}

/* Private user code ---------------------------------------------------------*/
void _adcDataReset(void) {
	adc_context.adc_read = false;
	adc_context.adc_cnt = 0;
	adc_context.adc_sum = 0;
}

HAL_StatusTypeDef _adcSendCmd(adcCmdId_t cmd) {
	HAL_StatusTypeDef ret = HAL_OK;

	uint16_t tx_buff[6] = { 0x00, };
	int16_t rx_buff[6] = { 0x00, };

	uint16_t send_cmd = 0x0000;
	uint16_t cmd_resp = 0;
	uint16_t crc = 0;

	switch (cmd) {
	case ADC_CMD_ID_RESET:
		send_cmd = ADC_CMD_RESET;
		cmd_resp = ADC_CMD_RESP_RESET;
		LogInfo("ADC Send Command Reset: %04X", send_cmd); //0011 - 17
		break;

	case ADC_CMD_ID_STANDBY:
		send_cmd = ADC_CMD_STANDBY;
		cmd_resp = ADC_CMD_RESP_STANDBY;
		adc_context.state = ADC_STATE_STANDBY;
		LogInfo("ADC Send Command Standby: %04X", send_cmd); // 0022 - 34
		break;

	case ADC_CMD_ID_WAKEUP:
		send_cmd = ADC_CMD_WAKEUP;
		cmd_resp = ADC_CMD_RESP_WAKEUP;
		if (adc_context.global_chop == false) {
			adc_context.state = ADC_STATE_CONTINOUS;
		} else {
			adc_context.state = ADC_STATE_GLOBAL_CHOP;
		}
		LogInfo("ADC Send Command Wakeup: %04X", send_cmd); //
		break;

	case ADC_CMD_ID_LOCK:
		send_cmd = ADC_CMD_LOCK;
		cmd_resp = ADC_CMD_RESP_LOCK;
		adc_context.lock = true;
		LogInfo("ADC Send Command Lock: %04X", send_cmd);
		break;

	case ADC_CMD_ID_UNLOCK:
		send_cmd = ADC_CMD_UNLOCK;
		cmd_resp = ADC_CMD_RESP_UNLOCK;
		adc_context.lock = false;
		LogInfo("ADC Send Command Unlock: %04X", send_cmd);
		break;

	default:
		ret = HAL_ERROR;
		LogInfo("Not supported command %04X", send_cmd);
		break;
	}

	/*
	 * Make CMD Array: [CMD] [CRC]
	 * */
	tx_buff[0] = send_cmd;
	tx_buff[1] = crc;

	if (ret == HAL_OK) {
		ret = _adcTransmitReceive(&tx_buff[0], &rx_buff[0], 6);
	}

	if (ret == HAL_OK) {
		if (cmd_resp == rx_buff[0]) {
			ret = HAL_OK;
			LogInfo("ADC Command valid response: %04X", rx_buff[0]);
		} else {
			ret = HAL_ERROR;
			LogError("ADC Command wrong response: %04X", rx_buff[0]);
		}
	} else {
		LogError("ADC Command response receive failed");
	}

	return ret;
}

HAL_StatusTypeDef _adcGetData(bool first_read) {
	HAL_StatusTypeDef ret = HAL_OK;

	uint16_t tx_buff[6] = { 0x00, };
	int16_t rx_buff[6] = { 0x00, };
	static float lsb = 0;
	static uint8_t data_idx = 0;
	float adc_result = 0;
	int32_t adc_result2 = 0;
	float adc_avr = 0;
	int32_t adc_avr2 = 0;

	tx_buff[0] = ADC_CMD_NULL;
	ret = _adcTransmitReceive(&tx_buff[0], &rx_buff[0], 6); //3

	if (ret == HAL_OK) {
		LogInfo("Get Ch Data: %04X %04X %04X %04X %04X", rx_buff[0], rx_buff[1],
				rx_buff[2], rx_buff[3], rx_buff[4]); /* [STATUS] [CH0 DATA] [CH1 DATA] */

		if (first_read == false) {
			adc_result = (float) ((rx_buff[data_idx]) * lsb); //(int16_t)
			adc_result2 = (int32_t) ((rx_buff[data_idx]) * lsb * 1000); //(int16_t)
			LogInfo("ADC Result: %d mV", adc_result2);

			adc_context.adc_sum += adc_result;

			if (++adc_context.adc_cnt >= adc_context.adc_num) {
				adc_avr = (float) adc_context.adc_sum / adc_context.adc_cnt;
				adc_avr2 =
						(int32_t) ((adc_context.adc_sum / adc_context.adc_cnt)
								* 1000);
				LogInfo("ADC Average: %d mV", adc_avr2);
				adc_context.adc_sum = 0;

				adc_context.cb_fn(adc_avr);
				adc_context.adc_read = false;
				adc_context.adc_cnt = 0;
			}
		} else {
			lsb =
					(float) 1200
							/ pow(2,
									adc_context.ch_cfg[adc_context.adc_cfg.ch].gain_mode)
							/ pow(2, 15);
			if (adc_context.adc_cfg.ch == ADC_CH_0) {
				data_idx = 1;
			} else if (adc_context.adc_cfg.ch == ADC_CH_1) {
				data_idx = 2;
			} else if (adc_context.adc_cfg.ch == ADC_CH_2) {
				data_idx = 3;
			} else if (adc_context.adc_cfg.ch == ADC_CH_3) {
				data_idx = 4;
			}
		}
	} else {
		LogError("Read Status and ADC Result failed");
	}

	return ret;
}

/*
 HAL_StatusTypeDef _adcGetData2(bool first_read)
 {
 HAL_StatusTypeDef ret = HAL_OK;

 uint16_t tx_buff[6] = {0x00,};
 uint16_t rx_buff[6] = {0x00,};
 static float lsb = 0;
 static uint8_t data_idx = 0;
 float adc_result = 0;
 int32_t adc_result2 = 0;
 float adc_avr = 0;
 int32_t adc_avr2 = 0;

 tx_buff[0] = ADC_CMD_NULL;
 ret = _adcTransmitReceive(&tx_buff[0], &rx_buff[0], 6); //3

 if (ret == HAL_OK)
 {
 LogInfo("Get Ch Data: %04X %04X %04X %04X %04X", rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4]); // [STATUS] [CH0 DATA] [CH1 DATA]

 if (first_read == false)
 {
 adc_result = (float)((rx_buff[data_idx]) * lsb); //(int16_t)
 adc_result2 = (int32_t)((rx_buff[data_idx]) * lsb * 1000); //(int16_t)
 LogInfo("ADC Result: %d mV", adc_result2);

 adc_context.adc_sum += adc_result;

 if (++adc_context.adc_cnt >= adc_context.adc_num)
 {
 adc_avr = (float)adc_context.adc_sum / adc_context.adc_cnt;
 adc_avr2 = (int32_t)((adc_context.adc_sum / adc_context.adc_cnt)*1000);
 LogInfo("ADC Average: %d mV", adc_avr2);
 adc_context.adc_sum = 0;

 adc_context.cb_fn(adc_avr);
 adc_context.adc_read = false;
 adc_context.adc_cnt = 0;
 }
 }
 else
 {
 lsb = (float)2400 / pow(2, adc_context.ch_cfg[adc_context.adc_cfg.ch].gain_mode) / pow(2, 16);
 if (adc_context.adc_cfg.ch == ADC_CH_0)
 {
 data_idx = 1;
 }
 else if (adc_context.adc_cfg.ch == ADC_CH_1)
 {
 data_idx = 2;
 }
 else if (adc_context.adc_cfg.ch == ADC_CH_2)
 {
 data_idx = 3;
 }
 else if (adc_context.adc_cfg.ch == ADC_CH_3)
 {
 data_idx = 4;
 }
 }
 }
 else
 {
 LogError("Read Status and ADC Result failed");
 }

 return ret;
 }
 */

HAL_StatusTypeDef _adcSetClockCfg(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	uint16_t send_cfg = 0x00;

	/* Ch Enable */
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_0].enable, ADC_CH0_EN_BCNT,
			ADC_CH0_EN_BOFF);
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_1].enable, ADC_CH1_EN_BCNT,
			ADC_CH1_EN_BOFF);
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_2].enable, ADC_CH2_EN_BCNT,
			ADC_CH2_EN_BOFF);
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_3].enable, ADC_CH3_EN_BCNT,
			ADC_CH3_EN_BOFF);

	/* Clock Select */
	send_cfg |= BF_VAL(adc_context.clock_sel, ADC_CLK_SEL_BCNT,
			ADC_CLK_SEL_BOFF);

	/* OSR Config */
	send_cfg |= BF_VAL(adc_context.osr_mode, ADC_OSR_MODE_BCNT,
			ADC_OSR_MODE_BOFF);

	/* Power mode Config */
	send_cfg |= BF_VAL(adc_context.pwr_mode, ADC_PWR_MODE_BCNT,
			ADC_PWR_MODE_BOFF);

	ret = _adcWriteRegister(ADC_REG_ADDR_CLOCK, send_cfg);

	if (ret == HAL_OK) {
		LogInfo("ADC Set Clock Config Success: %04X", send_cfg); //0F0E - 3854
	} else {
		LogInfo("ADC Set Clock Config Failed");
	}

	return ret;
}

HAL_StatusTypeDef _adcSetGainCfg(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	uint16_t send_cfg = 0x00;

	/* Clock Config */
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_0].gain_mode,
			ADC_GAIN_CFG_CH0_BCNT, ADC_GAIN_CFG_CH0_BOFF);
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_1].gain_mode,
			ADC_GAIN_CFG_CH1_BCNT, ADC_GAIN_CFG_CH1_BOFF);
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_2].gain_mode,
			ADC_GAIN_CFG_CH2_BCNT, ADC_GAIN_CFG_CH2_BOFF);
	send_cfg |= BF_VAL(adc_context.ch_cfg[ADC_CH_3].gain_mode,
			ADC_GAIN_CFG_CH3_BCNT, ADC_GAIN_CFG_CH3_BOFF);

	ret = _adcWriteRegister(ADC_REG_ADDR_GAIN, send_cfg);

	if (ret == HAL_OK) {
		LogInfo("ADC Set Gain Config Success: %04X", send_cfg); //0000
	} else {
		LogInfo("ADC Set Gain Config Failed");
	}

	return ret;
}

HAL_StatusTypeDef _adcSetChMuxCfg(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	uint16_t send_cfg = 0x00;
	uint8_t input_mode[ADC_CH_NUM] = { 0, };

	input_mode[0] = adc_context.ch_cfg[ADC_CH_0].input_mode;
	input_mode[1] = adc_context.ch_cfg[ADC_CH_1].input_mode;
	input_mode[2] = adc_context.ch_cfg[ADC_CH_2].input_mode;
	input_mode[3] = adc_context.ch_cfg[ADC_CH_3].input_mode;

	/* Ch MUX Config */
	for (uint8_t idx = 0; idx < 4; idx++) //5
			{
		send_cfg = BF_VAL(input_mode[idx], ADC_CH_CFG_MUX_BCNT,
				ADC_CH_CFG_MUX_BOFF);
		ret = _adcWriteRegister(ADC_REG_ADDR_CH0_CFG + 5 * idx, send_cfg);

		if (ret == HAL_OK) {
			LogInfo("ADC Set MUX CH%d Config Success: %04X", idx, send_cfg); // CH1234 success : 0000
		} else {
			LogError("ADC Set MUX CH%d Config Failed");
			break;
		}
	}

	return ret;
}

HAL_StatusTypeDef _adcWriteRegister(uint16_t reg, uint16_t data) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint16_t send_data[3] = { 0x00, };
	int16_t resp_data = 0x00;
	uint16_t save_data = 0x00;
	uint16_t resp_chk;

	send_data[0] = ADC_CMD_WREG_HEAD;
	send_data[0] |= BF_VAL(reg, ADC_CMD_W_R_REG_ADDR_BCNT,
			ADC_CMD_W_R_REG_ADDR_BOFF);
	send_data[1] = data;

	ret = _adcTransmit(&send_data[0], 3);

	if (ret == HAL_OK) {
		ret = _adcReceive(&resp_data, 1);
	}

	if (ret == HAL_OK) {
		resp_chk = BF_GET(resp_data, ADC_CMD_W_R_REG_ADDR_BCNT,
				ADC_CMD_W_R_REG_ADDR_BOFF);
		if (reg != resp_chk) {
			LogError("Register Write failed, cmd don't match: %04X | %04X", reg,
					resp_chk);

			ret = HAL_ERROR;
		}

		resp_chk = BF_GET(resp_data, ADC_CMD_W_R_DATA_NUM_BCNT,
				ADC_CMD_W_R_DATA_NUM_BOFF);
		if (resp_chk != 0) {
			LogError("Register Write failed, len don't match: 0 | %d",
					resp_chk);

			ret = HAL_ERROR;
		}
	}

	if (ret == HAL_OK) {
		ret = _adcReadRegister(reg, 1, &save_data);

		if (data != save_data) {
			LogError("Write & Read data don't match: %04X | %04X", data,
					save_data);

			ret = HAL_ERROR;
		}
	}

	return ret;
}

HAL_StatusTypeDef _adcReadRegister(uint16_t reg, uint8_t len, uint16_t *p_data) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t read_len = 0;
	uint16_t send_data[ADC_SPI_WORD_MIN_BUFF_SIZE] = { 0x00, };
	int16_t read_data[ADC_SPI_WORD_MAX_BUFF_SIZE] = { 0x00, };

	if (len > ADC_CMD_RREG_MAX_LEN) {
		read_len = ADC_CMD_RREG_MAX_LEN - 1;
		LogWarn("Read len changed: %d -> %d", read_len, ADC_CMD_RREG_MAX_LEN);
	} else if (len == 0) {
		read_len = 0;
		LogWarn("Read len changed: %d -> %d", read_len, 1);
	} else {
		read_len = len - 1;
	}

	send_data[0] = ADC_CMD_RREG_HEAD;
	send_data[0] |= BF_VAL(reg, ADC_CMD_W_R_REG_ADDR_BCNT,
			ADC_CMD_W_R_REG_ADDR_BOFF);
	send_data[0] |= BF_VAL(read_len, ADC_CMD_W_R_DATA_NUM_BCNT,
			ADC_CMD_W_R_DATA_NUM_BOFF);

	ret = _adcTransmit(&send_data[0], 2);

	if (ret == HAL_OK) {
		ret = _adcReceive(&read_data[0], len);
	}

	if (ret == HAL_OK) {
		if (len > 1) {
			/* [ACK] [DATA 1] [DATA 2] ... [DATA N] [CRC] */
			memcpy(p_data, &read_data[1], 2 * (len + 2));
		} else {
			memcpy(p_data, &read_data[0], 2);
		}
	}

	return ret;
}

#define USE_SPI_TXRX_FUNC	0
HAL_StatusTypeDef _adcTransmitReceive(uint16_t *p_tx_data, int16_t *p_rx_data,
		uint8_t len) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t tx_buff[ADC_SPI_WORD_MIN_BUFF_SIZE] = { 0, };
	uint8_t rx_buff[ADC_SPI_WORD_MAX_BUFF_SIZE] = { 0, };
	uint8_t word_size = adc_context.word_size.byte_num;

	if (adc_context.spi_handle != NULL) {
		/* Make Packet */
		_makeWordPacket(p_tx_data, len, &tx_buff[0]);

		/* Send Data */
		ADC_SPI_Select();
#if USE_SPI_TXRX_FUNC == 1
		ret = HAL_SPI_TransmitReceive(adc_context.spi_handle, &tx_buff[0], &rx_buff[0], word_size * len, 500);
#else
		ret = HAL_SPI_Transmit(adc_context.spi_handle, &tx_buff[0],
				word_size * len, 500);
		ADC_SPI_Deselect();
#endif
	} else {
		ret = HAL_ERROR;
	}

#if USE_SPI_TXRX_FUNC == 1
#else
	if (ret == HAL_OK) {
		ADC_SPI_Select();
		ret = HAL_SPI_Receive(adc_context.spi_handle, &rx_buff[0],
				word_size * len, 500);
	}
#endif

	if (ret == HAL_OK) {
		/* Parse Packet */
		_parseWordPacket(&rx_buff[0], len, p_rx_data);
	}

	ADC_SPI_Deselect();

	return ret;
}

HAL_StatusTypeDef _adcTransmit(uint16_t *p_data, uint8_t len) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t tx_buff[ADC_SPI_WORD_MIN_BUFF_SIZE] = { 0, };
	uint8_t word_size = adc_context.word_size.byte_num;

	if (adc_context.spi_handle != NULL) {
		/* Make Packet */
		_makeWordPacket(p_data, len, &tx_buff[0]);

		/* Send Data */
		ADC_SPI_Select();
		ret = HAL_SPI_Transmit(adc_context.spi_handle, &tx_buff[0],
				word_size * len, 500);
	} else {
		ret = HAL_ERROR;
	}

	ADC_SPI_Deselect();

	return ret;
}

HAL_StatusTypeDef _adcReceive(int16_t *p_data, uint8_t len) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t rx_buff[ADC_SPI_WORD_MAX_BUFF_SIZE] = { 0, };
	uint8_t word_size = adc_context.word_size.byte_num;

	if (adc_context.spi_handle != NULL) {
		/* Read Data */
		ADC_SPI_Select();
		ret = HAL_SPI_Receive(adc_context.spi_handle, &rx_buff[0],
				word_size * len, 500);

		/* Parse Packet */
		_parseWordPacket(&rx_buff[0], len, p_data);
	} else {
		ret = HAL_ERROR;
	}

	ADC_SPI_Deselect();

	return ret;
}

void _makeWordPacket(uint16_t *p_data, uint8_t len, uint8_t *p_result) {
	uint8_t word_size = adc_context.word_size.byte_num;

	for (uint8_t idx = 0; idx < len; idx++) {
		p_result[word_size * idx] = (p_data[idx] >> 8) & 0xFF;
		p_result[word_size * idx + 1] = p_data[idx] & 0xFF;

		/* padding.. */
	}
}

void _parseWordPacket(uint8_t *p_data, uint8_t len, int16_t *p_result) {
	uint8_t word_size = adc_context.word_size.byte_num;

	for (uint8_t idx = 0; idx < len; idx++) {
		p_result[idx] = (((p_data[word_size * idx] << 8) & 0xFF00)
				| (p_data[word_size * idx + 1] & 0xFF));
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ADC_DRDY__Pin) {
		if (adc_context.adc_read == true) {
			_adcGetData(false);
		}
	}
}
