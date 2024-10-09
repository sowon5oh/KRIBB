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
    MEAS_STATE_STANDBY = 0, /* Wail for start command */
    MEAS_STATE_LED_ON,
    MEAS_STATE_ADC,
    MEAS_STATE_ERROR,
    MEAS_STATE_MAX
} measState_t;

typedef struct {
    bool task_init;
    measState_t meas_state;
} measTaskContext_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void _meas_task_init(void);
static void _meas_set_init(void);
static void _meas_result_init(void);

static HAL_StatusTypeDef _meas_op_start(MeasSetChVal_t ch);
static void _meas_task_req_cb(void);

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
    .meas_state = MEAS_STATE_STANDBY, };
static MeasSetData_t meas_set_data;
static MeasResultData_t meas_result_data;
static MeasReqStatus_t meas_req_status_data;
static uint16_t recv_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];
static uint16_t monitor_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];

/* Public user code ----------------------------------------------------------*/
void Task_Meas_Init(void) {
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

HAL_StatusTypeDef Task_Meas_Request(MeasSetChVal_t ch) {
    //Single Read
    _meas_op_start(ch);

    //Multi Read
    //    for (uint8_t ch_idx = 0; ch_idx <= MEAS_SET_CH_3; ch_idx++) {
    //        if (MEAS_TARGET_CH_ACTIVE == meas_req_status_data.target_ch[ch_idx]) {
    //            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(ch_idx, HAL_LED_ON));
    //        }
    //        else {
    //            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(ch_idx, HAL_LED_SET_OFF));
    //        }
    //    }

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_RequestResult(MeasSetChVal_t *p_ch, MeasResultData_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_ch);
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    bool checked = false;

    for (uint8_t ch_idx = 0; ch_idx <= MEAS_SET_CH_3; ch_idx++) {
        if (MEAS_TARGET_CH_ACTIVE == meas_req_status_data.target_ch[ch_idx]) {
            checked = true;
            break;
        }
    }

    if (checked) {
        p_ch = meas_req_status_data.target_ch;
        memcpy(p_data, &meas_result_data, sizeof(MeasResultData_t));
        return HAL_OK;
    }
    else {
        SYS_LOG_ERR("Selected nothing");
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef Task_Meas_Get_Result(MeasResultCat_t result_cat, MeasSetChVal_t ch, uint16_t *p_result_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_TRUE(result_cat <= MEAS_RESULT_CAT_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_result_val);
    
    switch (result_cat) {
        case MEAS_RESULT_CAT_TEMP_ADC:
            return _meas_get_temperature_data();

        case MEAS_RESULT_CAT_RECV_PD_ADC:
            return _meas_get_recv_pd_data(ch);

        case MEAS_RESULT_CAT_MONITOR_PD_ADC:
            return _meas_get_monitor_pd_data(ch);

        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef Task_Meas_Get_Status(MeasReqStatus_t *p_status_val) {
    SYS_VERIFY_PARAM_NOT_NULL(p_status_val);

    memcpy(p_status_val, &meas_req_status_data, sizeof(MeasReqStatus_t));

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Ctrl_Led(MeasSetChVal_t ch, uint8_t *p_set_val) {
    uint8_t set_val;

    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);
    set_val = p_set_val[0];
    
    switch (ch) {
        case MEAS_SET_CH_1:
            return Hal_Led_Ctrl(HAL_LED_CH_1, set_val);

        case MEAS_SET_CH_2:
            return Hal_Led_Ctrl(HAL_LED_CH_2, set_val);

        case MEAS_SET_CH_3:
            return Hal_Led_Ctrl(HAL_LED_CH_3, set_val);

        case MEAS_SET_CH_ALL:
            return Hal_Led_Ctrl(HAL_LED_CH_1, set_val);
            return Hal_Led_Ctrl(HAL_LED_CH_2, set_val);
            return Hal_Led_Ctrl(HAL_LED_CH_3, set_val);

        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef Task_Meas_Ctrl_Monitor(MeasSetChVal_t ch, uint8_t *p_set_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);
    
    if (MEAS_STATE_STANDBY != meas_task_context.meas_state) {
        SYS_LOG_ERR("Privious measure not conmplted");
        return HAL_ERROR;
    }

    switch (ch) {
        case MEAS_SET_CH_1:
            meas_req_status_data.target_ch[MEAS_SET_CH_1] = MEAS_TARGET_CH_ACTIVE;
            meas_req_status_data.target_ch[MEAS_SET_CH_2] = MEAS_TARGET_CH_DEACTIV;
            meas_req_status_data.target_ch[MEAS_SET_CH_3] = MEAS_TARGET_CH_DEACTIV;
            break;

        case MEAS_SET_CH_2:
            meas_req_status_data.target_ch[MEAS_SET_CH_1] = MEAS_TARGET_CH_DEACTIV;
            meas_req_status_data.target_ch[MEAS_SET_CH_2] = MEAS_TARGET_CH_ACTIVE;
            meas_req_status_data.target_ch[MEAS_SET_CH_3] = MEAS_TARGET_CH_DEACTIV;
            break;

        case MEAS_SET_CH_3:
            meas_req_status_data.target_ch[MEAS_SET_CH_1] = MEAS_TARGET_CH_DEACTIV;
            meas_req_status_data.target_ch[MEAS_SET_CH_2] = MEAS_TARGET_CH_DEACTIV;
            meas_req_status_data.target_ch[MEAS_SET_CH_3] = MEAS_TARGET_CH_ACTIVE;
            break;

        case MEAS_SET_CH_ALL:
            meas_req_status_data.target_ch[MEAS_SET_CH_1] = MEAS_TARGET_CH_ACTIVE;
            meas_req_status_data.target_ch[MEAS_SET_CH_2] = MEAS_TARGET_CH_ACTIVE;
            meas_req_status_data.target_ch[MEAS_SET_CH_3] = MEAS_TARGET_CH_ACTIVE;
            break;

        default:
            return HAL_ERROR;
    }
    
    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
static void _meas_task_init(void) {
    meas_task_context.meas_state = MEAS_STATE_STANDBY;
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

static HAL_StatusTypeDef _meas_op_start(MeasSetChVal_t ch) {
    if (MEAS_STATE_STANDBY != meas_task_context.meas_state) {
        SYS_LOG_ERR("Privious measure not conmplted");
        return HAL_ERROR;
    }

    meas_task_context.meas_state = MEAS_STATE_ADC;
    
    for (uint8_t ch_idx = 0; ch_idx <= MEAS_SET_CH_3; ch_idx++) {
        meas_req_status_data.target_ch[ch_idx] = (ch == ch_idx) ? MEAS_TARGET_CH_ACTIVE : MEAS_TARGET_CH_DEACTIV;
    }

    SYS_LOG_INFO("[MEAS] Start led delay: %d msec", meas_set_data.adc_delay_ms[ch]);
    SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(ch, meas_set_data.led_on_level[ch]));
    App_Timer_Start(APP_TIMER_TYPE_MEAS, meas_set_data.led_on_time[ch], _meas_task_req_cb);
    
    return HAL_OK;
}

static void _meas_task_req_cb(void) {
    static MeasSetChVal_t ch;
    switch (meas_task_context.meas_state) {
        case MEAS_STATE_LED_ON:
            SYS_LOG_INFO("[MEAS] LED Time Complete. Start ADC Delay");
            /* LED Off */
            (void) Hal_Led_Ctrl(MEAS_SET_CH_1, HAL_LED_SET_OFF);
            (void) Hal_Led_Ctrl(MEAS_SET_CH_2, HAL_LED_SET_OFF);
            (void) Hal_Led_Ctrl(MEAS_SET_CH_3, HAL_LED_SET_OFF);

            for (ch = 0; ch <= MEAS_SET_CH_3; ch++) {
                if (MEAS_TARGET_CH_ACTIVE == meas_req_status_data.target_ch[ch]) {
                    break;
                }
            }
            App_Timer_Start(APP_TIMER_TYPE_MEAS, meas_set_data.adc_delay_ms[ch], _meas_task_req_cb);
            break;

        case MEAS_STATE_ADC:
            SYS_LOG_INFO("[MEAS] ADC Delay Complete. Read ADC");
            /* Read ADC */
            //TODO
            meas_task_context.meas_state = MEAS_STATE_STANDBY;

            SYS_LOG_INFO("[MEAS] ADC Read Complete. Send MMI Message");
            Task_MMI_SendMeasResult();
            break;

        default:
            break;
    }

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
