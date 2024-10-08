/**
 ******************************************************************************
 * @file           : task_meas.c
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
#include "app_util.h"
#include "task_meas.h"
#include "task_mmi.h"
#include "hal_drv_heater.h"
#include "hal_drv_led.h"
#include "hal_drv_pd.h"
#include "hal_drv_temperature.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    MEAS_STATE_WAIT_START_CMD = 0,
    MEAS_STATE_STANDBY,
    MEAS_STATE_MEAS,
    MEAS_STATE_DONE,
    MEAS_STATE_ERROR,
    MEAS_STATE_MAX
} meas_State_t;

typedef struct {
    bool task_init;
    bool meas_done;
    meas_State_t meas_state;
} measTaskContext_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void _meas_task_init(void);

static HAL_StatusTypeDef _meas_Operation(void);
static void _meas_state_changer(meas_State_t state);
static void _meas_task_req_cb(float result);

static void _meas_set_init(void);
static void _meas_result_init(void);

static void _meas_set_temp_ctrl_on(MeasSetChVal_t ch, MeasSetTempCtrlVal_t val);
static void _meas_set_led_on_time_ms(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_led_on_level(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_sample_cnt(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_delay_ms(MeasSetChVal_t ch, uint16_t val);
static HAL_StatusTypeDef _meas_get_temperature_data(void);
static HAL_StatusTypeDef _meas_get_recv_pd_data(MeasSetChVal_t ch);
static HAL_StatusTypeDef _meas_get_monitor_pd_data(MeasSetChVal_t ch);

/* Private variables ---------------------------------------------------------*/
static measTaskContext_t meas_task_context = {
    .task_init = false,
    .meas_state = MEAS_STATE_STANDBY,
    .meas_done =
    false, };
static MeasSetData_t meas_set_data;
static MeasResultData_t meas_result_data;
static uint16_t recv_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];
static uint16_t monitor_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];

/* Public user code ----------------------------------------------------------*/
void Task_Meas_init(void) {
    _meas_task_init();

    /* initialize data */
    _meas_set_init();
    _meas_result_init();

    /* send first protocol */
    Task_MMI_SendDeviceInfo();
}

void Task_Meas_Start(void) {
    //TODO
}

void Task_Meas_Stop(void) {
    //TODO
}

void Task_Meas_Process(void) {
    //TODO
}

HAL_StatusTypeDef Task_Meas_Apply_Set(MeasSetCat_t set_cat, MeasSetChVal_t ch, uint8_t *p_set_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_TRUE(set_cat <= MEAS_SET_CAT_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);

    switch (set_cat) {
        case MEAS_SET_CAT_TEMP_ON_OFF: {
            MeasSetTempCtrlVal_t temp_ctrl_val = (MeasSetTempCtrlVal_t) p_set_val[0];
            _meas_set_temp_ctrl_on(ch, temp_ctrl_val);
            break;
        }

        case MEAS_SET_CAT_LED_ON_TIME: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_led_on_time_ms(ch, set_val);
            break;
        }

        case MEAS_SET_CAT_LED_ON_LEVEL: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_led_on_level(ch, set_val);
            break;
        }

        case MEAS_SET_CAT_ADC_SAMPLE_CNT: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_adc_sample_cnt(ch, set_val);
            break;
        }

        case MEAS_SET_CAT_ADC_ON_DELAY: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_adc_delay_ms(ch, set_val);
            break;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_Set(MeasSetData_t *p_set_val) {
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);

    memcpy(p_set_val, &meas_set_data, sizeof(MeasSetData_t));

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_Result(MeasResultCat_t result_cat, MeasSetChVal_t ch, uint16_t *p_result_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_TRUE(result_cat <= MEAS_RESULT_CAT_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_result_val);

    switch (result_cat) {
        case MEAS_RESULT_CAT_TEMP_ADC:
            return _meas_get_temperature_data();
            break;

        case MEAS_RESULT_CAT_RECV_PD_ADC:
            return _meas_get_recv_pd_data(ch);
            break;

        case MEAS_RESULT_CAT_MONITOR_PD_ADC:
            return _meas_get_monitor_pd_data(ch);
            break;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Ctrl_Led(MeasSetChVal_t ch, uint8_t *p_set_val) {
    uint8_t set_val = *p_set_val;

    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);

    switch (ch) {
        case MEAS_SET_CH_1:
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_1, set_val))
            ;
            break;

        case MEAS_SET_CH_2:
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_2, set_val))
            ;
            break;

        case MEAS_SET_CH_3:
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_3, set_val))
            ;
            break;

        case MEAS_SET_CH_ALL:
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_ALL, set_val))
            ;
            break;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Ctrl_Monitor(MeasSetChVal_t ch, uint8_t *p_set_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);

    switch (ch) {
        case MEAS_SET_CH_1:
            //TODO
            break;

        case MEAS_SET_CH_2:
            break;

        case MEAS_SET_CH_3:
            break;

        case MEAS_SET_CH_ALL:
            break;
    }
    
    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
static void _meas_task_init(void) {
    meas_task_context.meas_state = MEAS_STATE_WAIT_START_CMD;
    meas_task_context.meas_done = false;
}

static void _meas_set_init(void) {
    _meas_set_temp_ctrl_on(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_TEMP_CTRL_ON);
    _meas_set_led_on_time_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_TIME_MS);
    _meas_set_led_on_level(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_LEVEL);
    _meas_set_adc_sample_cnt(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_SAMPLE_CNT);
    _meas_set_adc_delay_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_DELAY_MS);
}

static void _meas_result_init(void) {
    memset(&meas_result_data, 0, sizeof(MeasResultData_t));
}

static HAL_StatusTypeDef _meas_Operation(void) {
    HAL_StatusTypeDef ret = HAL_OK;

//TODO

    return ret;
}

static void _meas_state_changer(meas_State_t state) {
    if (meas_task_context.meas_state != state) {
        //        SYS_LOG_INFO("[MEAS STATE] %s ===> %s", meas_StateInfo[meas_task_context.meas_state].name, meas_StateInfo[state].name);
        SYS_LOG_INFO("---------------------------------");
        meas_task_context.meas_state = state;
    }
}

static void _meas_task_req_cb(float result) {
    meas_task_context.meas_done = true;
}

static void _meas_set_temp_ctrl_on(MeasSetChVal_t ch, MeasSetTempCtrlVal_t val) {
    switch (ch) {
        case MEAS_SET_CH_1:
            meas_set_data.temp_ctrl_on[CH1_IDX] = val;
            break;

        case MEAS_SET_CH_2:
            meas_set_data.temp_ctrl_on[CH2_IDX] = val;
            break;

        case MEAS_SET_CH_3:
            meas_set_data.temp_ctrl_on[CH3_IDX] = val;
            break;

        case MEAS_SET_CH_ALL:
            meas_set_data.temp_ctrl_on[CH1_IDX] = val;
            meas_set_data.temp_ctrl_on[CH2_IDX] = val;
            meas_set_data.temp_ctrl_on[CH3_IDX] = val;
            break;
    }

    SYS_LOG_DEBUG("Temperature Control On/Off settings: %d, %d, %d", meas_set_data.temp_ctrl_on[CH1_IDX], meas_set_data.temp_ctrl_on[CH2_IDX], meas_set_data.temp_ctrl_on[CH3_IDX]);
}

static void _meas_set_led_on_time_ms(MeasSetChVal_t ch, uint16_t val) {
    if (val > MEAS_SET_MAX_LED_ON_TIME_MS) {
        SYS_LOG_WARN("LED On time settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_MAX_LED_ON_TIME_MS;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }
    switch (ch) {
        case MEAS_SET_CH_1:
            meas_set_data.led_on_time[CH1_IDX] = val;
            break;

        case MEAS_SET_CH_2:
            meas_set_data.led_on_time[CH2_IDX] = val;
            break;

        case MEAS_SET_CH_3:
            meas_set_data.led_on_time[CH3_IDX] = val;
            break;

        case MEAS_SET_CH_ALL:
            meas_set_data.led_on_time[CH1_IDX] = val;
            meas_set_data.led_on_time[CH2_IDX] = val;
            meas_set_data.led_on_time[CH3_IDX] = val;
            break;
    }

    SYS_LOG_DEBUG("LED On time(ms) settings: %d, %d, %d", meas_set_data.led_on_time[CH1_IDX], meas_set_data.led_on_time[CH2_IDX], meas_set_data.led_on_time[CH3_IDX]);
}

static void _meas_set_led_on_level(MeasSetChVal_t ch, uint16_t val) {
    if (val > MEAS_SET_MAX_LED_ON_LEVEL) {
        SYS_LOG_WARN("LED On level settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_MAX_LED_ON_LEVEL;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }
    switch (ch) {
        case MEAS_SET_CH_1:
            meas_set_data.led_on_level[CH1_IDX] = val;
            break;

        case MEAS_SET_CH_2:
            meas_set_data.led_on_level[CH2_IDX] = val;
            break;

        case MEAS_SET_CH_3:
            meas_set_data.led_on_level[CH3_IDX] = val;
            break;

        case MEAS_SET_CH_ALL:
            meas_set_data.led_on_level[CH1_IDX] = val;
            meas_set_data.led_on_level[CH2_IDX] = val;
            meas_set_data.led_on_level[CH3_IDX] = val;
            break;
    }

    SYS_LOG_DEBUG("LED On level(Digit) settings: %d, %d, %d", meas_set_data.led_on_level[CH1_IDX], meas_set_data.led_on_level[CH2_IDX], meas_set_data.led_on_level[CH3_IDX]);
}

static void _meas_set_adc_sample_cnt(MeasSetChVal_t ch, uint16_t val) {
    if (val > MEAS_SET_MAX_ADC_SAMPLE_CNT) {
        SYS_LOG_WARN("ADC sample count settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_MAX_ADC_SAMPLE_CNT;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }
    switch (ch) {
        case MEAS_SET_CH_1:
            meas_set_data.adc_sample_cnt[CH1_IDX] = val;
            break;

        case MEAS_SET_CH_2:
            meas_set_data.adc_sample_cnt[CH2_IDX] = val;
            break;

        case MEAS_SET_CH_3:
            meas_set_data.adc_sample_cnt[CH3_IDX] = val;
            break;

        case MEAS_SET_CH_ALL:
            meas_set_data.adc_sample_cnt[CH1_IDX] = val;
            meas_set_data.adc_sample_cnt[CH2_IDX] = val;
            meas_set_data.adc_sample_cnt[CH3_IDX] = val;
            break;
    }

    SYS_LOG_DEBUG("ADC Sample count settings: %d, %d, %d", meas_set_data.adc_sample_cnt[CH1_IDX], meas_set_data.adc_sample_cnt[CH2_IDX], meas_set_data.adc_sample_cnt[CH3_IDX]);
}

static void _meas_set_adc_delay_ms(MeasSetChVal_t ch, uint16_t val) {
    if (val > MEAS_SET_MAX_ADC_DELAY_MS) {
        SYS_LOG_WARN("ADC delay time settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_MAX_ADC_DELAY_MS;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }
    switch (ch) {
        case MEAS_SET_CH_1:
            meas_set_data.adc_delay_ms[CH1_IDX] = val;
            break;

        case MEAS_SET_CH_2:
            meas_set_data.adc_delay_ms[CH2_IDX] = val;
            break;

        case MEAS_SET_CH_3:
            meas_set_data.adc_delay_ms[CH3_IDX] = val;
            break;

        case MEAS_SET_CH_ALL:
            meas_set_data.adc_delay_ms[CH1_IDX] = val;
            meas_set_data.adc_delay_ms[CH2_IDX] = val;
            meas_set_data.adc_delay_ms[CH3_IDX] = val;
            break;
    }

    SYS_LOG_DEBUG("ADC Delay time(ms) settings: %d, %d, %d", meas_set_data.adc_delay_ms[CH1_IDX], meas_set_data.adc_delay_ms[CH2_IDX], meas_set_data.adc_delay_ms[CH3_IDX]);
}

static HAL_StatusTypeDef _meas_get_temperature_data(void) {
    HalTempData_t temp_data_buff;

    SYS_VERIFY_SUCCESS(Hal_Temp_GetData(&temp_data_buff));
    meas_result_data.temperature_data[CH1_IDX] = temp_data_buff.ch1_temp;
    meas_result_data.temperature_data[CH2_IDX] = temp_data_buff.ch2_temp;
    meas_result_data.temperature_data[CH3_IDX] = temp_data_buff.ch3_temp;

    return HAL_OK;
}

static HAL_StatusTypeDef _meas_get_recv_pd_data(MeasSetChVal_t ch) {
    uint8_t ch_idx;
    uint8_t data_idx;
    uint8_t fail_cnt;

    if (MEAS_SET_CH_ALL == ch) {
        for (ch_idx = 0; ch_idx < MEAS_SET_CH_MAX; ch_idx++) {
            if (meas_set_data.adc_sample_cnt[ch_idx] == 0) {
                SYS_LOG_ERR("Sampling num 0 Error");
                return HAL_ERROR;
            }

            SYS_LOG_DEBUG("Recv PD [%d]ch: sampling num: [%d]", ch_idx, meas_set_data.adc_sample_cnt[ch_idx]);
            fail_cnt = 0;
            for (data_idx = 0; data_idx < meas_set_data.adc_sample_cnt[ch_idx]; data_idx++) {
                if (HAL_OK != Hal_Pd_GetRecvData(ch_idx, &recv_pd_buff[ch_idx][data_idx])) {
                    if (++fail_cnt > (meas_set_data.adc_sample_cnt[ch_idx] / 2)) {
                        SYS_LOG_ERR("Recv PD [%d]ch: Data collection failed. Fail count: %d", ch_idx, fail_cnt);
                        return HAL_ERROR;
                    }
                }
            }
            meas_result_data.recv_pd_data[ch_idx] = ARRAY_AVERAGE(recv_pd_buff[ch_idx], meas_set_data.adc_sample_cnt[ch_idx]);
        }
    }
    else {
        if (meas_set_data.adc_sample_cnt[ch] == 0) {
            SYS_LOG_ERR("Sampling num 0 Error");
            return HAL_ERROR;
        }
        
        SYS_LOG_DEBUG("Recv PD [%d]ch: sampling num: [%d]", ch, meas_set_data.adc_sample_cnt[ch]);
        fail_cnt = 0;
        for (data_idx = 0; data_idx < meas_set_data.adc_sample_cnt[ch]; data_idx++) {
            if (HAL_OK != Hal_Pd_GetRecvData(ch, &recv_pd_buff[ch][data_idx])) {
                if (++fail_cnt > (meas_set_data.adc_sample_cnt[ch] / 2)) {
                    SYS_LOG_ERR("Recv PD [%d]ch: Data collection failed. Fail count: %d", ch, fail_cnt);
                    return HAL_ERROR;
                }
            }
        }
        meas_result_data.recv_pd_data[ch] = ARRAY_AVERAGE(recv_pd_buff[ch], meas_set_data.adc_sample_cnt[ch]);
    }
    
    return HAL_OK;
}

static HAL_StatusTypeDef _meas_get_monitor_pd_data(MeasSetChVal_t ch) {
    uint8_t ch_idx;
    uint8_t data_idx;
    uint8_t fail_cnt;
    
    if (MEAS_SET_CH_ALL == ch) {
        for (ch_idx = 0; ch_idx < MEAS_SET_CH_MAX; ch_idx++) {
            if (meas_set_data.adc_sample_cnt[ch_idx] == 0) {
                SYS_LOG_ERR("Sampling num 0 Error");
                return HAL_ERROR;
            }
            
            SYS_LOG_DEBUG("Monitor PD [%d]ch: sampling num: [%d]", ch_idx, meas_set_data.adc_sample_cnt[ch_idx]);
            fail_cnt = 0;
            for (data_idx = 0; data_idx < meas_set_data.adc_sample_cnt[ch_idx]; data_idx++) {
                if (HAL_OK != Hal_Pd_GetMonitorData(ch_idx, &monitor_pd_buff[ch_idx][data_idx])) {
                    if (++fail_cnt > (meas_set_data.adc_sample_cnt[ch_idx] / 2)) {
                        SYS_LOG_ERR("Monitor PD [%d]ch: Data collection failed. Fail count: %d", ch_idx, fail_cnt);
                        return HAL_ERROR;
                    }
                }
            }
            meas_result_data.monitor_pd_data[ch_idx] = ARRAY_AVERAGE(monitor_pd_buff[ch_idx], meas_set_data.adc_sample_cnt[ch_idx]);
        }
    }
    else {
        if (meas_set_data.adc_sample_cnt[ch] == 0) {
            SYS_LOG_ERR("Sampling num 0 Error");
            return HAL_ERROR;
        }
        
        SYS_LOG_DEBUG("Monitor PD [%d]ch: sampling num: [%d]", ch, meas_set_data.adc_sample_cnt[ch]);
        fail_cnt = 0;
        for (data_idx = 0; data_idx < meas_set_data.adc_sample_cnt[ch]; data_idx++) {
            if (HAL_OK != Hal_Pd_GetMonitorData(ch, &monitor_pd_buff[ch][data_idx])) {
                if (++fail_cnt > (meas_set_data.adc_sample_cnt[ch] / 2)) {
                    SYS_LOG_ERR("Monitor PD [%d]ch: Data collection failed. Fail count: %d", ch, fail_cnt);
                    return HAL_ERROR;
                }
            }
        }
        meas_result_data.monitor_pd_data[ch] = ARRAY_AVERAGE(monitor_pd_buff[ch], meas_set_data.adc_sample_cnt[ch]);
    }
    
    return HAL_OK;
}
