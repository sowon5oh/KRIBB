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
#include "task_temp_ctrl.h"
#include "task_mmi.h"
#include "hal_drv_heater.h"
#include "hal_drv_led.h"
#include "hal_drv_pd.h"
#include "hal_drv_temperature.h"
#include "hal_drv_fram.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    MEAS_STATE_LED_ON = 0,
    MEAS_STATE_LED_STABLE,
    MEAS_STATE_ADC,
    MEAS_STATE_ADC_DONE,
    MEAS_STATE_ERROR,
    MEAS_STATE_START = MEAS_STATE_LED_ON,
    MEAS_STATE_STOP = MEAS_STATE_ADC,
    MEAS_STATE_MAX = MEAS_STATE_ERROR,
} measState_t;

typedef struct {
    bool task_op_state;
    measState_t meas_state;

    MeasCh_t meas_cur_ch;
    uint8_t sample_cnt;
} measTaskContext_t;

/* Private define ------------------------------------------------------------*/
#define MEAS_TASK_DUTY_MS            10
#define MEAS_TASK_LED_STABLE_TIME_MS 100
#define MEAS_TASK_ADC_READ_TIME_MS   100

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Init */
static void _meas_task_init(void);
static void _meas_task_enable(bool enable);
static void _meas_set_init(void);
static void _meas_result_init(void);
/* Timer Callback */
static void _meask_task_led_stable_cb(void);
static void _meask_task_adc_cb(void);
/* Task Callback */
static void _meas_task_cb(void);
/* Function */
static void _meas_set_temp_ctrl_type(MeasSetChVal_t ch, MeasSetTempCtrlTypeVal_t val);
static void _meas_set_led_on_time_ms(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_led_on_level(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_sample_cnt(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_delay_ms(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_stable_temperature_degree(uint16_t val);
static void _meas_get_temperature_data(void);
static void _led_ctrl(MeasCh_t ch, uint16_t set_data);
int16_t _calc_pd_avr(int16_t *p_buff, uint16_t sample_cnt);

/* Private variables ---------------------------------------------------------*/
static measTaskContext_t meas_task_context = {
    .task_op_state = false,
    .meas_state = MEAS_STATE_START,
    .meas_cur_ch = CH1_IDX,
    .sample_cnt = 0, };
static MeasSetData_t meas_set_data;
static MeasResultData_t meas_result_data;
static MeasReqStatus_t meas_req_status_data;
static int16_t recv_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT] = {
    0, };
static int16_t monitor_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT] = {
    0, };

#if(FEATURE_SETTINGS_DEFAULT == 1)
static MeasSetData_t meas_set_default_data = {
    .temp_ctrl_type[CH1_IDX] = MEAS_SET_DEFAULT_temp_ctrl_type,
    .temp_ctrl_type[CH2_IDX] = MEAS_SET_DEFAULT_temp_ctrl_type,
    .temp_ctrl_type[CH3_IDX] = MEAS_SET_DEFAULT_temp_ctrl_type,
    .led_on_time[CH1_IDX] = MEAS_SET_DEFAULT_LED_ON_TIME_MS,
    .led_on_time[CH2_IDX] = MEAS_SET_DEFAULT_LED_ON_TIME_MS,
    .led_on_time[CH3_IDX] = MEAS_SET_DEFAULT_LED_ON_TIME_MS,
    .led_on_level[CH1_IDX] = MEAS_SET_DEFAULT_LED_ON_LEVEL,
    .led_on_level[CH2_IDX] = MEAS_SET_DEFAULT_LED_ON_LEVEL,
    .led_on_level[CH3_IDX] = MEAS_SET_DEFAULT_LED_ON_LEVEL,
    .adc_sample_cnt[CH1_IDX] = MEAS_SET_DEFAULT_ADC_SAMPLE_CNT,
    .adc_sample_cnt[CH2_IDX] = MEAS_SET_DEFAULT_ADC_SAMPLE_CNT,
    .adc_sample_cnt[CH3_IDX] = MEAS_SET_DEFAULT_ADC_SAMPLE_CNT,
    .adc_delay_ms[CH1_IDX] = MEAS_SET_DEFAULT_ADC_DELAY_MS,
    .adc_delay_ms[CH2_IDX] = MEAS_SET_DEFAULT_ADC_DELAY_MS,
    .adc_delay_ms[CH3_IDX] = MEAS_SET_DEFAULT_ADC_DELAY_MS,
    .stable_temperature = MEAS_SET_DEFAULT_STABLE_TEMPERATURE_DEGREE, };
#endif

/* Public user code ----------------------------------------------------------*/
void Task_Meas_Init(void) {
    /* Sensor Init */
    SYS_VERIFY_SUCCESS_VOID(Hal_Heater_Init());
    SYS_VERIFY_SUCCESS_VOID(Hal_Led_Init(&hi2c2)); /* dac */
    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Init(&hi2c1)); /* fram */
    SYS_VERIFY_SUCCESS_VOID(Hal_Temp_Init(&hadc1)); /* adc */
    SYS_VERIFY_SUCCESS_VOID(Hal_Pd_Init(&hspi1, NULL)); /* adc */

    _meas_task_init();
    
    /* initialize data */
    _meas_set_init();
    _meas_result_init();

    /* send first protocol */
    Task_MMI_SendDeviceInfo();
}

HAL_StatusTypeDef Task_Meas_Apply_Set(MeasSetCat_t set_cat, MeasSetChVal_t ch, uint8_t *p_set_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_TRUE(set_cat <= MEAS_SET_CAT_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);
    
    /* Pause Measure */
    _meas_task_enable(false);

    /* Setting Value Apply */
    switch (set_cat) {
        case MEAS_SET_CAT_TEMP_ON_OFF: {
            MeasSetTempCtrlTypeVal_t temp_ctrl_val = (MeasSetTempCtrlTypeVal_t) p_set_val[0];
            _meas_set_temp_ctrl_type(ch, temp_ctrl_val);
            Hal_Fram_Write(FRAM_TEMP_SETTING_ADDR_CH1 + (ch - 1) * 2, FRAM_TEMP_SETTING_SINGLE_DATA_LEN, &temp_ctrl_val);
            break;
        }

        case MEAS_SET_CAT_LED_ON_TIME: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_led_on_time_ms(ch, set_val);
            Hal_Fram_Write(FRAM_LED_ON_TIME_ADDR_CH1 + (ch - 1) * 2, FRAM_LED_ON_TIME_SINGLE_DATA_LEN, (uint8_t*) &set_val);
            break;
        }

        case MEAS_SET_CAT_LED_ON_LEVEL: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_led_on_level(ch, set_val);
            Hal_Fram_Write(FRAM_LED_ON_LEVEL_ADDR_CH1 + (ch - 1) * 2, FRAM_LED_ON_LEVEL_SINGLE_DATA_LEN, (uint8_t*) &set_val);
            break;
        }

        case MEAS_SET_CAT_ADC_SAMPLE_CNT: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_adc_sample_cnt(ch, set_val);
            Hal_Fram_Write(FRAM_ADC_SAMPLE_CNT_ADDR_CH1 + (ch - 1) * 2, FRAM_ADC_SAMPLE_CNT_SINGLE_DATA_LEN, (uint8_t*) &set_val);
            break;
        }

        case MEAS_SET_CAT_ADC_ON_DELAY: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_adc_delay_ms(ch, set_val);
            Hal_Fram_Write(FRAM_ADC_DELAY_MS_ADDR_CH1 + (ch - 1) * 2, FRAM_ADC_DELAY_MS_SINGLE_DATA_LEN, (uint8_t*) &set_val);
            break;
        }

        case MEAS_SET_CAT_STABLE_TEMPERATURE: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_stable_temperature_degree(set_val);
            Hal_Fram_Write(FRAM_STABLE_TEMPERATURE_ADDR, FRAM_STABLE_TEMPERATURE_SINGLE_DATA_LEN, (uint8_t*) &set_val);
            break;
        }
    }

    /* Restart Measure */
    _meas_task_enable(true);

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_Set(MeasSetData_t *p_set_val) {
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);
    
    memcpy(p_set_val, &meas_set_data, sizeof(MeasSetData_t));
    
    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Start(void) {
    if (MEAS_STATE_LED_ON != meas_task_context.meas_state) {
        SYS_LOG_ERR("Privious measure not conmplted, Please Stop");
        return HAL_ERROR;
    }

#if 0
    for (uint8_t ch_idx = 0; ch_idx < CH_NUM; ch_idx++) {
        meas_req_status_data.target_ch[ch_idx] = (ch == ch_idx) ? MEAS_TARGET_CH_ACTIVE : MEAS_TARGET_CH_DEACTIV;
    }
#else
    /* All Ch Activated */
    meas_req_status_data.target_ch[CH1_IDX] = MEAS_TARGET_CH_ACTIVE;
    meas_req_status_data.target_ch[CH2_IDX] = MEAS_TARGET_CH_ACTIVE;
    meas_req_status_data.target_ch[CH3_IDX] = MEAS_TARGET_CH_ACTIVE;
#endif

    _meas_task_enable(true);

    SYS_LOG_INFO("[MEASURE START]");
    SYS_LOG_INFO("- LED ON TIME MS    : %3d / %3d / %3d", meas_set_data.led_on_time[CH1_IDX], meas_set_data.led_on_time[CH2_IDX], meas_set_data.led_on_time[CH3_IDX]);
    SYS_LOG_INFO("- LED ON LEVEL      : %4d / %4d / %4d", meas_set_data.led_on_level[CH1_IDX], meas_set_data.led_on_level[CH2_IDX], meas_set_data.led_on_level[CH3_IDX]);
    SYS_LOG_INFO("- ADC DELAY MS      : %4d / %4d / %4d", meas_set_data.adc_delay_ms[CH1_IDX], meas_set_data.adc_delay_ms[CH2_IDX], meas_set_data.adc_delay_ms[CH3_IDX]);
    SYS_LOG_INFO("- ADC SAMPLE CNT    : %2d / %2d / %2d", meas_set_data.adc_sample_cnt[CH1_IDX], meas_set_data.adc_sample_cnt[CH2_IDX], meas_set_data.adc_sample_cnt[CH3_IDX]);
    SYS_LOG_INFO("- STABLE TEMPERATURE: %d", meas_set_data.stable_temperature);

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Stop(void) {
    _meas_task_enable(false);

    SYS_LOG_INFO("[MEASURE STOP]");

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_AllResult(MeasResultData_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    memcpy(p_data, &meas_result_data, sizeof(MeasResultData_t));

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_Result(MeasResultCat_t result_cat, MeasSetChVal_t ch, uint16_t *p_result_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_TRUE(result_cat <= MEAS_RESULT_CAT_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_result_val);
    
    switch (result_cat) {
        case MEAS_RESULT_CAT_TEMPERATURE:
            _meas_get_temperature_data();
            memcpy(p_result_val, &meas_result_data.temperature_data, sizeof(uint16_t) * CH_NUM);
            return HAL_OK;

        case MEAS_RESULT_CAT_RECV_PD_ADC:
            memcpy(p_result_val, &meas_result_data.recv_pd_data, sizeof(uint16_t) * CH_NUM);
            return HAL_OK;

        case MEAS_RESULT_CAT_MONITOR_PD_ADC:
            memcpy(p_result_val, &meas_result_data.monitor_pd_data, sizeof(uint16_t) * CH_NUM);
            return HAL_OK;

        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef Task_Meas_Get_Status(MeasReqStatus_t *p_status_val) {
    SYS_VERIFY_PARAM_NOT_NULL(p_status_val);

    memcpy(p_status_val, &meas_req_status_data, sizeof(MeasReqStatus_t));

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Ctrl_Led(MeasSetChVal_t ch, MeasSetLedForcedCtrlVal_t ctrl) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);

    uint8_t levels[3] = {
        CH1_IDX,
        CH2_IDX,
        CH3_IDX };
    
    switch (ch) {
        case MEAS_SET_CH_1:
        case MEAS_SET_CH_2:
        case MEAS_SET_CH_3:
            _led_ctrl(HAL_LED_CH_1 + (ch - MEAS_SET_CH_1), ctrl == LED_CTRL_FORCE_ON ? meas_set_data.led_on_level[levels[ch - MEAS_SET_CH_1]] : HAL_LED_LEVEL_OFF);
            meas_set_data.led_ctrl_on[levels[ch - MEAS_SET_CH_1]] = ctrl;
            break;

        case MEAS_SET_CH_ALL:
            for (uint8_t i = 0; i < 3; i++) {
                _led_ctrl(HAL_LED_CH_1 + i, ctrl == LED_CTRL_FORCE_ON ? meas_set_data.led_on_level[i] : HAL_LED_LEVEL_OFF);
                meas_set_data.led_ctrl_on[i] = ctrl;
            }
            break;

        default:
            SYS_LOG_ERR("Invalid LED Control: ch=%d, ctrl=%d", ch, ctrl);
            return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Ctrl_Monitor(MeasSetChVal_t ch, uint8_t *p_set_val) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_set_val);
    
    /* Pause Measure */
    _meas_task_enable(false);

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
    
    /* Restart Measure */
    _meas_task_enable(true);

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
static void _meas_task_init(void) {
    meas_task_context.meas_state = MEAS_STATE_START;
}

static void _meas_task_enable(bool enable) {
    if (enable) {
        meas_task_context.task_op_state = true;
        meas_task_context.meas_state = MEAS_STATE_START;

        /* Init Result Data */
        _meas_result_init();

        /* Task Start */
        App_Task_Start(APP_TASK_MEASURE, MEAS_TASK_DUTY_MS, _meas_task_cb);
    }
    else {
        meas_task_context.task_op_state = false;
        meas_task_context.meas_state = MEAS_STATE_STOP;

        /* Task Stop */
        App_Task_Stop(APP_TASK_MEASURE);

        /* All LED Off */
        _led_ctrl(CH1_IDX, 0);
        _led_ctrl(CH2_IDX, 0);
        _led_ctrl(CH3_IDX, 0);
    }
}

static void _meas_set_init(void) {
    uint8_t read_data[FRAM_DATA_MAX_LEN] = {
        0, };

    Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);

#if(FEATURE_SETTINGS_DEFAULT == 1)
    /* Set default */
    _meas_set_temp_ctrl_type(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_temp_ctrl_type);
    _meas_set_led_on_time_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_TIME_MS);
    _meas_set_led_on_level(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_LEVEL);
    _meas_set_adc_sample_cnt(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_SAMPLE_CNT);
    _meas_set_adc_delay_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_DELAY_MS);
    _meas_set_stable_temperature_degree(MEAS_SET_DEFAULT_STABLE_TEMPERATURE_DEGREE);

    /* Save Fram */
    Hal_Fram_Write(FRAM_TEMP_SETTING_ADDR, FRAM_TEMP_SETTING_DATA_LEN, (uint8_t*) meas_set_default_data.temp_ctrl_type);
    Hal_Fram_Write(FRAM_LED_ON_TIME_ADDR, FRAM_LED_ON_TIME_DATA_LEN, (uint8_t*) meas_set_default_data.led_on_time);
    Hal_Fram_Write(FRAM_LED_ON_LEVEL_ADDR, FRAM_LED_ON_LEVEL_DATA_LEN, (uint8_t*) meas_set_default_data.led_on_level);
    Hal_Fram_Write(FRAM_ADC_SAMPLE_CNT_ADDR, FRAM_ADC_SAMPLE_CNT_DATA_LEN, (uint8_t*) meas_set_default_data.adc_sample_cnt);
    Hal_Fram_Write(FRAM_ADC_DELAY_MS_ADDR, FRAM_ADC_DELAY_MS_DATA_LEN, (uint8_t*) meas_set_default_data.adc_delay_ms);
    Hal_Fram_Write(FRAM_STABLE_TEMPERATURE_ADDR, FRAM_STABLE_TEMPERATURE_DATA_LEN, (uint8_t*) &meas_set_default_data.stable_temperature);

    Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data); /* for check */
#else
    /* Set read data */
    _meas_set_temp_ctrl_type(MEAS_SET_CH_1, read_data[FRAM_TEMP_SETTING_ADDR_CH1]);
    _meas_set_temp_ctrl_type(MEAS_SET_CH_2, read_data[FRAM_TEMP_SETTING_ADDR_CH2]);
    _meas_set_temp_ctrl_type(MEAS_SET_CH_3, read_data[FRAM_TEMP_SETTING_ADDR_CH3]);

    _meas_set_led_on_time_ms(MEAS_SET_CH_1, read_data[FRAM_LED_ON_TIME_ADDR_CH1 + 1] << 8 | read_data[FRAM_LED_ON_TIME_ADDR_CH1]);
    _meas_set_led_on_time_ms(MEAS_SET_CH_2, read_data[FRAM_LED_ON_TIME_ADDR_CH2 + 1] << 8 | read_data[FRAM_LED_ON_TIME_ADDR_CH2]);
    _meas_set_led_on_time_ms(MEAS_SET_CH_3, read_data[FRAM_LED_ON_TIME_ADDR_CH3 + 1] << 8 | read_data[FRAM_LED_ON_TIME_ADDR_CH3]);

    _meas_set_led_on_level(MEAS_SET_CH_1, read_data[FRAM_LED_ON_LEVEL_ADDR_CH1 + 1] << 8 | read_data[FRAM_LED_ON_LEVEL_ADDR_CH1]);
    _meas_set_led_on_level(MEAS_SET_CH_2, read_data[FRAM_LED_ON_LEVEL_ADDR_CH2 + 1] << 8 | read_data[FRAM_LED_ON_LEVEL_ADDR_CH2]);
    _meas_set_led_on_level(MEAS_SET_CH_3, read_data[FRAM_LED_ON_LEVEL_ADDR_CH3 + 1] << 8 | read_data[FRAM_LED_ON_LEVEL_ADDR_CH3]);

    _meas_set_adc_sample_cnt(MEAS_SET_CH_1, read_data[FRAM_ADC_SAMPLE_CNT_ADDR_CH1 + 1] << 8 | read_data[FRAM_ADC_SAMPLE_CNT_ADDR_CH1]);
    _meas_set_adc_sample_cnt(MEAS_SET_CH_2, read_data[FRAM_ADC_SAMPLE_CNT_ADDR_CH2 + 1] << 8 | read_data[FRAM_ADC_SAMPLE_CNT_ADDR_CH2]);
    _meas_set_adc_sample_cnt(MEAS_SET_CH_3, read_data[FRAM_ADC_SAMPLE_CNT_ADDR_CH3 + 1] << 8 | read_data[FRAM_ADC_SAMPLE_CNT_ADDR_CH3]);

    _meas_set_adc_delay_ms(MEAS_SET_CH_1, read_data[FRAM_ADC_DELAY_MS_ADDR_CH1 + 1] << 8 | read_data[FRAM_ADC_DELAY_MS_ADDR_CH1]);
    _meas_set_adc_delay_ms(MEAS_SET_CH_2, read_data[FRAM_ADC_DELAY_MS_ADDR_CH2 + 1] << 8 | read_data[FRAM_ADC_DELAY_MS_ADDR_CH2]);
    _meas_set_adc_delay_ms(MEAS_SET_CH_3, read_data[FRAM_ADC_DELAY_MS_ADDR_CH3 + 1] << 8 | read_data[FRAM_ADC_DELAY_MS_ADDR_CH3]);

    _meas_set_stable_temperature_degree(read_data[FRAM_STABLE_TEMPERATURE_ADDR + 1] << 8 | read_data[FRAM_STABLE_TEMPERATURE_ADDR]);

#endif

    SYS_LOG_INFO("Settings Done");
}

static void _meas_result_init(void) {
    memset(&meas_result_data, 0, sizeof(MeasResultData_t));
    memset(&recv_pd_buff, 0, sizeof(recv_pd_buff));
    memset(&monitor_pd_buff, 0, sizeof(monitor_pd_buff));
}

static void _meask_task_led_stable_cb(void) {
    MeasCh_t cur_ch = meas_task_context.meas_cur_ch;

//    if (HAL_OK == Hal_Pd_SetMonitorCh(cur_ch)) {
        if (HAL_OK == Hal_Pd_Start()) {
            meas_task_context.meas_state = MEAS_STATE_ADC;
            App_Timer_Start(APP_TIMER_ID_ADC_DELAY, meas_set_data.adc_delay_ms[cur_ch], true, _meask_task_adc_cb);
        }
        else {
            meas_task_context.meas_state = MEAS_STATE_ERROR;
            SYS_LOG_ERR("Pd Start Failed");
        }
//    }
//    else {
//        meas_task_context.meas_state = MEAS_STATE_ERROR;
//        SYS_LOG_ERR("Monitor Pd Ch Select Failed");
//    }
}

static void _meask_task_adc_cb(void) {
    MeasCh_t cur_ch = meas_task_context.meas_cur_ch;

    Hal_Pd_GetRecvData(cur_ch, &recv_pd_buff[cur_ch][meas_task_context.sample_cnt]);
    Hal_Pd_GetMonitorData(cur_ch, &monitor_pd_buff[cur_ch][meas_task_context.sample_cnt]);

    if (++meas_task_context.sample_cnt >= meas_set_data.adc_sample_cnt[cur_ch]) {
        meas_result_data.recv_pd_data[cur_ch] = _calc_pd_avr(recv_pd_buff[cur_ch], meas_set_data.adc_sample_cnt[cur_ch]);
        meas_result_data.monitor_pd_data[cur_ch] = _calc_pd_avr(monitor_pd_buff[cur_ch], meas_set_data.adc_sample_cnt[cur_ch]);
        meas_task_context.meas_state = MEAS_STATE_ADC_DONE;

        /* ADC Stop */
        Hal_Pd_Stop();
    }
}

static void _meas_task_cb(void) {
    MeasCh_t cur_ch = meas_task_context.meas_cur_ch;

    switch (meas_task_context.meas_state) {
        case MEAS_STATE_LED_ON:
            if (meas_req_status_data.target_ch[cur_ch] == MEAS_TARGET_CH_ACTIVE) {
                /* LED On */
                _led_ctrl(cur_ch, meas_set_data.led_on_level[cur_ch]);
                meas_task_context.meas_state = MEAS_STATE_LED_STABLE;
                App_Timer_Start(APP_TIMER_ID_LED_STABLE, MEAS_TASK_LED_STABLE_TIME_MS, false, _meask_task_led_stable_cb);

                SYS_LOG_INFO("-----------------[ CH %d Measure Start ]-----------------", cur_ch + 1);
            }
            else {
                meas_task_context.meas_state = MEAS_STATE_ADC_DONE;
                SYS_LOG_INFO("-----------------[ CH %d Pass ]-----------------", cur_ch + 1);
            }
            break;

        case MEAS_STATE_LED_STABLE:
            /* Waiting Stable Time */
            break;

        case MEAS_STATE_ADC:
            /* Collecting Data */
            break;
            
        case MEAS_STATE_ADC_DONE:
            SYS_LOG_INFO("-----------------[ CH Measure Stop ]-----------------", cur_ch + 1);

            /* Calculate Average */
            Hal_Pd_GetRecvData(cur_ch, &meas_result_data.recv_pd_data[cur_ch]);
            Hal_Pd_GetMonitorData(cur_ch, &meas_result_data.monitor_pd_data[cur_ch]);

            /* Save result */
            SYS_LOG_INFO("- receive pd : %d", meas_result_data.recv_pd_data[cur_ch]);
            SYS_LOG_INFO("- monitor pd : %d", meas_result_data.monitor_pd_data[cur_ch]);
            SYS_LOG_INFO("-----------------------------------------------------");
            
            /* LED Off */
            _led_ctrl(cur_ch, 0);

            /* Change Channel */
            if (++cur_ch >= CH_NUM) {
                meas_task_context.meas_cur_ch = CH1_IDX;
            }
            else {
                meas_task_context.meas_cur_ch = cur_ch;
            }

            /* Send MMI */
            //TODO
            /* Restart Routine */
            meas_task_context.meas_state = MEAS_STATE_LED_ON;
            break;
            
        case MEAS_STATE_ERROR:
            SYS_LOG_ERR("[MEAS] Error State");
            return;
    }
}

static void _meas_set_temp_ctrl_type(MeasSetChVal_t ch, MeasSetTempCtrlTypeVal_t val) {
    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.temp_ctrl_type[CH1_IDX] = val;
        meas_set_data.temp_ctrl_type[CH2_IDX] = val;
        meas_set_data.temp_ctrl_type[CH3_IDX] = val;
        Task_TempCtrl_SetCtrlType(CH1_IDX, val);
        Task_TempCtrl_SetCtrlType(CH2_IDX, val);
        Task_TempCtrl_SetCtrlType(CH3_IDX, val);
    }
    else {
        meas_set_data.temp_ctrl_type[ch] = val;
    }

    SYS_LOG_INFO("Temperature Control On/Off settings: %d, %d, %d", meas_set_data.temp_ctrl_type[CH1_IDX], meas_set_data.temp_ctrl_type[CH2_IDX], meas_set_data.temp_ctrl_type[CH3_IDX]);
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

    SYS_LOG_INFO("LED On time(ms) settings: %d, %d, %d", meas_set_data.led_on_time[CH1_IDX], meas_set_data.led_on_time[CH2_IDX], meas_set_data.led_on_time[CH3_IDX]);
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

    SYS_LOG_INFO("LED On level(Digit) settings: %d, %d, %d", meas_set_data.led_on_level[CH1_IDX], meas_set_data.led_on_level[CH2_IDX], meas_set_data.led_on_level[CH3_IDX]);
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

    SYS_LOG_INFO("ADC Sample count settings: %d, %d, %d", meas_set_data.adc_sample_cnt[CH1_IDX], meas_set_data.adc_sample_cnt[CH2_IDX], meas_set_data.adc_sample_cnt[CH3_IDX]);
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

    SYS_LOG_INFO("ADC Delay time(ms) settings: %d, %d, %d", meas_set_data.adc_delay_ms[CH1_IDX], meas_set_data.adc_delay_ms[CH2_IDX], meas_set_data.adc_delay_ms[CH3_IDX]);
}

static void _meas_set_stable_temperature_degree(uint16_t val) {
    if (val > (MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE)) {
        SYS_LOG_WARN("stable temperature settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }
    else if (val < ( MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE)) {
        SYS_LOG_WARN("stable temperature settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }
    
    meas_set_data.stable_temperature = val;
    Task_TempCtrl_SetStableTemp((float) val / MEAS_SET_TEMPERATURE_DEGREE_SCALE);
    
    SYS_LOG_INFO("Stable Temperature setting: %d(/100) 'C)", meas_set_data.stable_temperature);
}

static void _meas_get_temperature_data(void) {
    float temp_data_buff;
    
    Task_TempCtrl_GetCurTemp(CH1_IDX, &temp_data_buff);
    meas_result_data.temperature_data[CH1_IDX] = (int16_t) (temp_data_buff * MEAS_SET_TEMPERATURE_DEGREE_SCALE);
    Task_TempCtrl_GetCurTemp(CH2_IDX, &temp_data_buff);
    meas_result_data.temperature_data[CH2_IDX] = (int16_t) (temp_data_buff * MEAS_SET_TEMPERATURE_DEGREE_SCALE);
    Task_TempCtrl_GetCurTemp(CH3_IDX, &temp_data_buff);
    meas_result_data.temperature_data[CH3_IDX] = (int16_t) (temp_data_buff * MEAS_SET_TEMPERATURE_DEGREE_SCALE);
}

void _led_ctrl(MeasCh_t ch, uint16_t set_data) {
    bool led_status = (set_data > 0) ? true : false;
    
    meas_req_status_data.led_on_status[ch] = led_status;
    SYS_VERIFY_SUCCESS_VOID(Hal_Led_Ctrl((HalLedCh_t )ch, set_data));
}

int16_t _calc_pd_avr(int16_t *p_buff, uint16_t sample_cnt) {
    int32_t sum = 0;
    int16_t result = 0;
    
    if (sample_cnt > 0) {
        for (uint16_t idx = 0; idx < sample_cnt; idx++) {
            sum += p_buff[idx];
        }
        result = (int16_t) ((double) sum / sample_cnt); // 정밀도 손실 방지
        return result;
    }
    else {
        return 0;
    }
}
