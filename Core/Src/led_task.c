/**
 ******************************************************************************
 * @file           : led_task.c
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
#include "led_task.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	LED_MEAS_MODE_1 = 0, LED_MEAS_MODE_3,
} ledMeasMode_t;
typedef enum {
	LED_TASK_STATE_IDLE = 0, LED_TASK_STATE_RUN, LED_TASK_STATE_MAX
} ledTaskState_t;

typedef enum {
	LED_MEAS_STATE_WAIT_START_CMD = 0,
	LED_MEAS_STATE_STANDBY,
	LED_MEAS_STATE_MEAS,
	LED_MEAS_STATE_DONE,
	LED_MEAS_STATE_ERROR,
	LED_MEAS_STATE_MAX
} ledMeasState_t;

typedef struct {
	uint8_t id;
	char name[10];
} ledStateName_t;

typedef struct {
	uint16_t time_cnt;
	uint8_t schedule_idx;

	MeasAvrReq_t meas_cfg;
	bool meas_done;
	MeasSchedule_t led_schedule[LED_SCHEDULE_LEN_MAX];
	uint8_t led_schedule_len;

	MeasM1Result_t m1_result[LED_SCHEDULE_LEN_MAX];
	MeasM3Result_t m3_result;

	ledMeasMode_t meas_mode;
	ledTaskState_t task_state;
	ledMeasState_t meas_state;
} ledTaskContext_t;

/* Private define ------------------------------------------------------------*/
#define LED_MEAS_CFG_DEFAULT_ADC_CH 			ADC_CH_0
#define LED_MEAS_CFG_DEFAULT_WAIT_TIME_MS		400
#define LED_MEAS_CFG_DEFAULT_MEAS_TIMEOUT_MS	5000
#define LED_MEAS_CFG_DEFAULT_MEAS_NUM			100
#define LED_MEAS_CFG_DEFAULT_SPS				ADC_SPS_32K

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef _ledMeasOperation(void);
HAL_StatusTypeDef _ledScheduleSet(MeasSchedule_t *p_schdeule,
		uint8_t schedule_len);
void _taskStateChanger(ledTaskState_t state);
void _measStateChanger(ledMeasState_t state);
void _ledOnOff(DAC_chSel_t led_ch, bool set);
void _ledTaskInit(void);
void _ledTaskMeasCfgInit(void);
void _ledTaskAvrReqCallback(float result);
void _opperateLEDOn(void);
void _opperateLEDOff(void);

/* Private variables ---------------------------------------------------------*/
ledTaskContext_t led_task = { .time_cnt = 0, .schedule_idx = 0, .meas_mode =
		LED_MEAS_MODE_1, .task_state = LED_TASK_STATE_IDLE, .meas_state =
		LED_MEAS_STATE_STANDBY, .meas_done = false, };

#define LED_INITIAL_VAL_B  0x985
#define LED_INITIAL_VAL_G  0x2A5
#define LED_INITIAL_VAL_R  0xD32
#define LED_INITIAL_VAL_O  0x54A

MeasSchedule_t led_schedule_default[LED_SCHEDULE_LEN_MAX] = { { DAC_CH_0,
LED_INITIAL_VAL_B }, /* CH0 1,080mV */
{ DAC_CH_1, LED_INITIAL_VAL_G }, /* CH1   300mV */
{ DAC_CH_2, LED_INITIAL_VAL_R }, /* CH2 1,497mV */
{ DAC_CH_3, LED_INITIAL_VAL_O }, /* CH3   600mV */
/* You can add or remove led schedules... */
};

ledStateName_t ledTaskStateInfo[LED_TASK_STATE_MAX] = { { LED_TASK_STATE_IDLE,
		"IDLE" }, { LED_TASK_STATE_RUN, "RUN" }, };

ledStateName_t ledMeasStateInfo[LED_MEAS_STATE_MAX] = { {
		LED_MEAS_STATE_STANDBY, "STANDBY" }, { LED_MEAS_STATE_MEAS, "MEAS" }, {
		LED_MEAS_STATE_DONE, "DONE" }, };

/* Public user code ----------------------------------------------------------*/
void LED_TASK_Init(void) {
	_ledTaskInit();
	_ledTaskMeasCfgInit();
}

HAL_StatusTypeDef LED_TASK_CFG_Change(MeasAvrReq_t *p_req_info) {
	HAL_StatusTypeDef ret = HAL_OK;

	if (p_req_info == NULL) {
		LogError("Invalid parameter");
		ret = HAL_ERROR;
	} else {
		led_task.meas_cfg.Mux_ch = p_req_info->Mux_ch; //Mux_ch
		led_task.meas_cfg.ch = p_req_info->ch; // Monitor signal measure // ADC_CH_3
		led_task.meas_cfg.sps = p_req_info->sps; //LED_MEAS_CFG_DEFAULT_SPS;
		led_task.meas_cfg.adc_num = p_req_info->adc_num; //sps LED_MEAS_CFG_DEFAULT_MEAS_NUM;
		led_task.meas_cfg.wait_time = p_req_info->wait_time; //LED_MEAS_CFG_DEFAULT_WAIT_TIME_MS;
	}

	return ret;
}

HAL_StatusTypeDef LED_TASK_SetSchedule(MeasSchedule_t *p_schdeule,
		uint8_t schedule_len) {
	HAL_StatusTypeDef ret = HAL_OK;

	if ((p_schdeule == NULL) | (schedule_len == 0)) {
		LogError("Invalid parameter");
		ret = HAL_ERROR;
	} else {
		ret = _ledScheduleSet(p_schdeule, schedule_len);
	}

	return ret;
}

HAL_StatusTypeDef LED_TASK_SetAdc(MeasAvrReq_t *p_req_info) {
	HAL_StatusTypeDef ret = HAL_OK;

	if (p_req_info == NULL) {
		LogError("Invalid parameter");
		ret = HAL_ERROR;
	} else {
		memcpy(&led_task.meas_cfg, p_req_info, sizeof(MeasAvrReq_t));
	}

	return ret;
}

void LED_TASK_Start(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	if (led_task.task_state != LED_TASK_STATE_RUN) {
		/* Activate ADC */
		ret = ADC_Start();

		if (ret == HAL_OK) {
			LogInfo("ADC Start Success");
			_taskStateChanger(LED_TASK_STATE_RUN);
//			_opperateLEDOff();
		} else {
			LogError("ADC Start Failed");
			_taskStateChanger(LED_TASK_STATE_IDLE);
		}
	}
}

void LED_TASK_Stop(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	if (led_task.task_state != LED_TASK_STATE_IDLE) {
		/* Stop ADC */
		ADC_MeasStop();

		/* Deactivate ADC */
		ret = ADC_Stop();

		if (ret == HAL_OK) {
			LogInfo("ADC Stop Success");
		} else {
			LogError("ADC Stop Failed");
		}

		_taskStateChanger(LED_TASK_STATE_IDLE);
	}

	_ledTaskInit();
	_ledOnOff(DAC_CH_ALL, false);
}

void LED_TASK_MEAS_M1_Start(void) {
	if (led_task.meas_mode != LED_MEAS_MODE_1) {
		/* Stop ADC */
		ADC_MeasStop();

		/* Reset Measure Task */
		_ledTaskInit();
		led_task.task_state = LED_TASK_STATE_RUN;
		led_task.meas_mode = LED_MEAS_MODE_1;

		/* Measure Setting for mode 1 */
		//led_task.meas_cfg.ch = ADC_CH_1; // Monitor signal measure // ADC_CH_3
		//led_task.meas_cfg.adc_num = LED_MEAS_CFG_DEFAULT_MEAS_NUM;
		//led_task.meas_cfg.wait_time = LED_MEAS_CFG_DEFAULT_WAIT_TIME_MS;
		//led_task.meas_cfg.sps = LED_MEAS_CFG_DEFAULT_SPS;
	}

	if (led_task.task_state == LED_TASK_STATE_RUN) {
		_measStateChanger(LED_MEAS_STATE_STANDBY);
	}
}

void LED_TASK_MEAS_M3_Start(void) {
	if (led_task.meas_mode != LED_MEAS_MODE_3) {
		/* Stop ADC */
		ADC_MeasStop();

		/* Reset Measure Task */
		_ledTaskInit();
		led_task.task_state = LED_TASK_STATE_RUN;
		led_task.meas_mode = LED_MEAS_MODE_3;

		/* Measure Setting for mode 3 */
		//led_task.meas_cfg.ch = ADC_CH_0; // Receive signal measure
		//led_task.meas_cfg.adc_num = LED_MEAS_CFG_DEFAULT_MEAS_NUM;
		//led_task.meas_cfg.wait_time = LED_MEAS_CFG_DEFAULT_WAIT_TIME_MS;
		//led_task.meas_cfg.sps = LED_MEAS_CFG_DEFAULT_SPS;
	}

	if (led_task.task_state == LED_TASK_STATE_RUN) {
		_measStateChanger(LED_MEAS_STATE_STANDBY);
	}
}

void LED_TASK(void) {
	switch (led_task.task_state) {
	case LED_TASK_STATE_IDLE:
		break;

	case LED_TASK_STATE_RUN:
		/* Timer Count */
		if (led_task.meas_state != LED_MEAS_STATE_WAIT_START_CMD) {
			led_task.time_cnt += 1;
		}

		/* Measure Operation */
		_ledMeasOperation();

		break;

	default:
		break;
	}
}

void LED_TASK_AdcCollectDone(void) {
	_measStateChanger(LED_MEAS_STATE_DONE);
}

/* Private user code ---------------------------------------------------------*/
HAL_StatusTypeDef _ledMeasOperation(void) {
	HAL_StatusTypeDef ret = HAL_OK;

	/******************************* [ MODE 1 ] *******************************/
	if (led_task.meas_mode == LED_MEAS_MODE_1) {
		switch (led_task.meas_state) {
		case LED_MEAS_STATE_WAIT_START_CMD:
			/* Wait for Command */
			break;

		case LED_MEAS_STATE_STANDBY:
			if (led_task.time_cnt < led_task.meas_cfg.wait_time) {
				/* LED ON */
				//				_opperateLEDOn();
				_ledOnOff(led_task.led_schedule[led_task.schedule_idx].led_ch,
				true);
				_measStateChanger(LED_MEAS_STATE_STANDBY);
			} else if (led_task.time_cnt >= led_task.meas_cfg.wait_time) {
				ret = ADC_ReqAvr(&led_task.meas_cfg, _ledTaskAvrReqCallback);

				_measStateChanger(LED_MEAS_STATE_MEAS);
				led_task.time_cnt = 0;

				LogInfo("LED SCHEDULE [%d] [LED CH: %d]", led_task.schedule_idx,
						led_task.led_schedule[led_task.schedule_idx].led_ch);
				//led_task.led_schedule[led_task.schedule_idx].led_ch
			}
			break;

		case LED_MEAS_STATE_MEAS:
			/* Wait for Measure Done Callback.. */
			if (led_task.time_cnt > LED_MEAS_CFG_DEFAULT_MEAS_TIMEOUT_MS) {
				LogError("Timeout");
				led_task.time_cnt = 0;

				_measStateChanger(LED_MEAS_STATE_ERROR);
			} else if (led_task.meas_done == true) {
				_measStateChanger(LED_MEAS_STATE_DONE);
				led_task.meas_done = false;
			}
			break;

		case LED_MEAS_STATE_DONE:
			/* LED OFF */
			_ledOnOff(led_task.led_schedule[led_task.schedule_idx].led_ch,
			false);

			/* Select Next Schedule */
			if (++led_task.schedule_idx >= led_task.led_schedule_len) {
				led_task.schedule_idx = 0;
				led_task.time_cnt = 0;

				/* Send Result */
				LogInfo("Send Result to MMI");
				MMI_M1ResultSender(&led_task.m1_result[0],
						led_task.led_schedule_len);

				memset(&led_task.m1_result[0], 0, sizeof(MeasM1Result_t));
				_measStateChanger(LED_MEAS_STATE_WAIT_START_CMD);
			} else {
				led_task.time_cnt = 0;
				_measStateChanger(LED_MEAS_STATE_STANDBY);
			}
//				_opperateLEDOff();
			break;

		case LED_MEAS_STATE_ERROR:
			LogError("ADC Measureing Failed, Go to initial state");
			_ledTaskInit();
			break;

		default:
			break;
		}
	}
	/******************************* [ MODE 3 ] *******************************/
	else if (led_task.meas_mode == LED_MEAS_MODE_3) {
		switch (led_task.meas_state) {
		case LED_MEAS_STATE_WAIT_START_CMD:
			/* Wait for Command */
			break;

		case LED_MEAS_STATE_STANDBY:
			ret = ADC_ReqAvr(&led_task.meas_cfg, _ledTaskAvrReqCallback);

			_measStateChanger(LED_MEAS_STATE_MEAS);
			led_task.time_cnt = 0;
			break;

		case LED_MEAS_STATE_MEAS:
			/* Wait for Measure Done Callback.. */
			if (led_task.time_cnt > LED_MEAS_CFG_DEFAULT_MEAS_TIMEOUT_MS) {
				LogError("Timeout");
				led_task.time_cnt = 0;

				_measStateChanger(LED_MEAS_STATE_ERROR);
			} else if (led_task.meas_done == true) {
				_measStateChanger(LED_MEAS_STATE_DONE);
				led_task.meas_done = false;
			}
			break;

		case LED_MEAS_STATE_DONE:
			/* Send Result */
			LogInfo("Send Result to MMI");
			MMI_M3ResultSender(&led_task.m3_result);

			led_task.m3_result.avr_adc = 0.0f;
			led_task.time_cnt = 0;
			_measStateChanger(LED_MEAS_STATE_WAIT_START_CMD);
			break;

		case LED_MEAS_STATE_ERROR:
			LogError("ADC Measureing Failed, Go to initial state");
			_ledTaskInit();
			break;

		default:
			break;
		}
	}

	return ret;
}

void _taskStateChanger(ledTaskState_t state) {
	if (led_task.task_state != state) {
		LogInfo("**********************************");
		LogInfo("[TASK STATE] %s ======> %s",
				ledTaskStateInfo[led_task.task_state].name,
				ledTaskStateInfo[state].name);
		LogInfo("**********************************");
		led_task.task_state = state;
	}
}

void _measStateChanger(ledMeasState_t state) {
	if (led_task.meas_state != state) {
		LogInfo("[MEAS STATE] %s ===> %s",
				ledMeasStateInfo[led_task.meas_state].name,
				ledMeasStateInfo[state].name);
		LogInfo("---------------------------------");
		led_task.meas_state = state;
	}
}

HAL_StatusTypeDef _ledScheduleSet(MeasSchedule_t *p_schdeule,
		uint8_t schedule_len) {
	HAL_StatusTypeDef ret = HAL_OK;

	uint8_t idx = 0;

	if (schedule_len > LED_SCHEDULE_LEN_MAX) {
		schedule_len = LED_SCHEDULE_LEN_MAX;
	}

	led_task.led_schedule_len = schedule_len;

	for (idx = 0; idx < schedule_len; idx++) {
		/* Copy Initial setting */
		led_task.led_schedule[idx].led_ch = p_schdeule[idx].led_ch;
		led_task.led_schedule[idx].led_data = p_schdeule[idx].led_data;
		LogInfo("%d LED Set to %04X", p_schdeule[idx].led_ch,
				p_schdeule[idx].led_data);

		/* DAC Setting */
		ret = DAC_SetData(led_task.led_schedule[idx].led_ch,
				led_task.led_schedule[idx].led_data);

		if (ret != HAL_OK) {
			break;
		}
	}

	return ret;
}

void _ledOnOff(DAC_chSel_t led_ch, bool led_cfg) {
	GPIO_PinState pin_state = RESET;
	static bool ch_on[DAC_CH_NUM] = { 0, };

	if (led_cfg == true) {
		//pin_state = GPIO_PIN_SET;
		pin_state = GPIO_PIN_RESET;
	} else {
		//pin_state = GPIO_PIN_RESET;
		pin_state = GPIO_PIN_SET;
	}

	if (led_ch != DAC_CH_ALL) {
		if (ch_on[led_ch] != led_cfg) {
			ch_on[led_ch] = led_cfg;
			LogDebug("LED %d CH: %d", led_ch, led_cfg);
		} else {
			return;
		}
	} else {
		memset(ch_on, led_cfg, sizeof(ch_on));
		LogInfo("LED ALL CH: %d", led_cfg);
	}

	switch (led_ch) {
	case DAC_CH_0: //CH1
		HAL_GPIO_WritePin(CH1_LED_CON_GPIO_Port, CH1_LED_CON_Pin, pin_state);
		break;

	case DAC_CH_1: //CH2
		HAL_GPIO_WritePin(CH2_LED_CON_GPIO_Port, CH2_LED_CON_Pin, pin_state);
		break;

	case DAC_CH_2: //CH3
		HAL_GPIO_WritePin(CH3_LED_CON_GPIO_Port, CH3_LED_CON_Pin, pin_state);
		break;

	case DAC_CH_3: //CH4
//		HAL_GPIO_WritePin(CH4_LED_CON_GPIO_Port, CH4_LED_CON_Pin, pin_state);
		if (led_task.meas_cfg.Mux_ch == 1) {
			HAL_GPIO_WritePin(CH1_LED_CON_GPIO_Port, CH1_LED_CON_Pin,
					pin_state);
		} else if (led_task.meas_cfg.Mux_ch == 2) {
			HAL_GPIO_WritePin(CH2_LED_CON_GPIO_Port, CH2_LED_CON_Pin,
					pin_state);
		} else if (led_task.meas_cfg.Mux_ch == 3) {
			HAL_GPIO_WritePin(CH3_LED_CON_GPIO_Port, CH3_LED_CON_Pin,
					pin_state);
		} else {

		}
		break;

	case DAC_CH_ALL:
		HAL_GPIO_WritePin(CH1_LED_CON_GPIO_Port, CH1_LED_CON_Pin, pin_state);
		HAL_GPIO_WritePin(CH2_LED_CON_GPIO_Port, CH2_LED_CON_Pin, pin_state);
		HAL_GPIO_WritePin(CH3_LED_CON_GPIO_Port, CH3_LED_CON_Pin, pin_state);
//		HAL_GPIO_WritePin(CH4_LED_CON_GPIO_Port, CH4_LED_CON_Pin, pin_state);
		break;

	default:
		break;
	}
}

void _ledTaskInit(void) {
	led_task.time_cnt = 0;
	led_task.schedule_idx = 0;

	//led_task.meas_mode = LED_MEAS_MODE_3; // for test
	led_task.meas_mode = LED_MEAS_MODE_1; //
	led_task.task_state = LED_TASK_STATE_IDLE;
	led_task.meas_state = LED_MEAS_STATE_WAIT_START_CMD;

	led_task.meas_done = false;

	_ledOnOff(DAC_CH_ALL, false);

	memset(&led_task.m1_result[0], 0, sizeof(MeasM1Result_t));
	led_task.m3_result.avr_adc = 0.0f;
}

void _ledTaskMeasCfgInit(void) {
	led_task.meas_cfg.ch = LED_MEAS_CFG_DEFAULT_ADC_CH;
	led_task.meas_cfg.adc_num = LED_MEAS_CFG_DEFAULT_MEAS_NUM;
	led_task.meas_cfg.wait_time = LED_MEAS_CFG_DEFAULT_WAIT_TIME_MS;
	led_task.meas_cfg.sps = LED_MEAS_CFG_DEFAULT_SPS;

	_ledScheduleSet(&led_schedule_default[0], LED_SCHEDULE_LEN_MAX);
}

void _ledTaskAvrReqCallback(float result) {
	led_task.meas_done = true;

	if (led_task.meas_mode == LED_MEAS_MODE_1) {
		led_task.m1_result[led_task.schedule_idx].led_ch =
				led_task.led_schedule[led_task.schedule_idx].led_ch;
		led_task.m1_result[led_task.schedule_idx].avr_adc = result;
	} else if (led_task.meas_mode == LED_MEAS_MODE_3) {
		led_task.m3_result.avr_adc = result;
	}
}

//void _opperateLEDOn(void)
//{
//	HAL_GPIO_WritePin(GREEN_STBY_LED_GPIO_Port, GREEN_STBY_LED_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(RED_OPP_LED_GPIO_Port, RED_OPP_LED_Pin, GPIO_PIN_SET);
//}
//
//void _opperateLEDOff(void)
//{
//	HAL_GPIO_WritePin(RED_OPP_LED_GPIO_Port, RED_OPP_LED_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GREEN_STBY_LED_GPIO_Port, GREEN_STBY_LED_Pin, GPIO_PIN_SET);
//}

