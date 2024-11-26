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
#include "hal_drv_fram.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    MEAS_STATE_STANDBY = 0, /* Wail for start command */
    MEAS_STATE_LED_ON,
    MEAS_STATE_ADC_REQ,
    MEAS_STATE_ADC_DONE,
    MEAS_STATE_SEND_RESULT,
    MEAS_STATE_ERROR,
    MEAS_STATE_MAX = MEAS_STATE_ERROR,
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
static void _meas_task_led_timeout_cb(void);
static void _meas_task_req_cb(void);

static void _meas_set_temp_ctrl_on(MeasSetChVal_t ch, MeasSetTempCtrlVal_t val);
static void _meas_set_led_on_time_ms(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_led_on_level(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_sample_cnt(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_delay_ms(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_stable_temperature_degree(uint16_t val);
static HAL_StatusTypeDef _meas_get_temperature_data(void);
static HAL_StatusTypeDef _meas_get_recv_pd_data(MeasSetChVal_t ch);
static HAL_StatusTypeDef _meas_get_monitor_pd_data(MeasSetChVal_t ch);
static void _heater_ctrl(void);
static void _led_ctrl(HalLedCh_t ch, uint16_t set_data);
int16_t _calc_pd_avr(int16_t *p_buff, uint16_t sample_cnt);

/* Private variables ---------------------------------------------------------*/
static measTaskContext_t meas_task_context = { .task_init = false, .meas_state = MEAS_STATE_STANDBY, };
static MeasSetData_t meas_set_data;
static MeasResultData_t meas_result_data;
static MeasReqStatus_t meas_req_status_data;
static int16_t recv_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];
static int16_t monitor_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];
static uint16_t temperature_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];
static uint16_t sample_idx;

/* Public user code ----------------------------------------------------------*/
void Task_Meas_Init(void) {
    /* Sensor Init */
    SYS_VERIFY_SUCCESS_VOID(Hal_Heater_Init());
    SYS_VERIFY_SUCCESS_VOID(Hal_Led_Init(&hi2c2)); /* dac */
    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Init(&hi2c1)); /* fram */
    SYS_VERIFY_SUCCESS_VOID(Hal_Temp_Init(&hadc1)); /* adc */
    SYS_VERIFY_SUCCESS_VOID(Hal_Pd_Init(&hspi1, _meas_task_req_cb)); /* adc */

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

            /* Get Monitor Pd data */
            _meas_get_monitor_pd_data(ch);

            /* Response to mmi */
            Task_MMI_SendMonitorPdResult(ch);
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

        case MEAS_SET_CAT_STABLE_TEMPERATURE: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_stable_temperature_degree(set_val);
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

HAL_StatusTypeDef Task_Meas_Request(MeasSetChVal_t meas_ch) {
    uint8_t ch;

    if (MEAS_STATE_STANDBY != meas_task_context.meas_state) {
        SYS_LOG_ERR("Privious measure not conmplted");
        return HAL_ERROR;
    }

    switch (meas_ch) {
        case MEAS_SET_CH_1:
            ch = CH1_IDX;
            break;

        case MEAS_SET_CH_2:
            ch = CH2_IDX;
            break;

        case MEAS_SET_CH_3:
            ch = CH3_IDX;
            break;

        case MEAS_SET_CH_ALL:
            SYS_LOG_ERR("All Ch Measure is TBD");
            return HAL_ERROR;

        default:
            SYS_LOG_ERR("Invalid Ch %d", meas_ch);
            return HAL_ERROR;
    }

    SYS_LOG_INFO("[MEASURE START]");
    SYS_LOG_INFO("- CH                : %d", meas_ch);
    SYS_LOG_INFO("- LED ON TIME MS    : %d", meas_set_data.led_on_time[ch]);
    SYS_LOG_INFO("- LED ON LEVEL      : %d", meas_set_data.led_on_level[ch]);
    SYS_LOG_INFO("- ADC DELAY MS      : %d", meas_set_data.adc_delay_ms[ch]);
    SYS_LOG_INFO("- ADC SAMPLE CNT    : %d", meas_set_data.adc_sample_cnt[ch]);
    SYS_LOG_INFO("- STABLE TEMPERATURE: %d", meas_set_data.stable_temperature);
    _meas_op_start(ch);

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
        case MEAS_RESULT_CAT_TEMPERATURE:
            SYS_VERIFY_SUCCESS(_meas_get_temperature_data());
            memcpy(p_result_val, &meas_result_data.temperature_data, sizeof(uint16_t) * CH_NUM);
            return HAL_OK;

        case MEAS_RESULT_CAT_RECV_PD_ADC:
            SYS_VERIFY_SUCCESS(_meas_get_recv_pd_data(ch));
            memcpy(p_result_val, &meas_result_data.recv_pd_data, sizeof(uint16_t) * CH_NUM);
            return HAL_OK;

        case MEAS_RESULT_CAT_MONITOR_PD_ADC:
            SYS_VERIFY_SUCCESS(_meas_get_monitor_pd_data(ch));
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
    
    switch (ch) {
        case MEAS_SET_CH_1:
            if (LED_CTRL_FORCE_ON == ctrl) {
                _led_ctrl(HAL_LED_CH_1, meas_set_data.led_on_level[CH1_IDX]);
            }
            else {
                _led_ctrl(HAL_LED_CH_1, HAL_LED_LEVEL_OFF);
            }
            meas_set_data.led_ctrl_on[CH1_IDX] = ctrl;
            break;

        case MEAS_SET_CH_2:
            if (LED_CTRL_FORCE_ON == ctrl) {
                _led_ctrl(HAL_LED_CH_2, meas_set_data.led_on_level[CH2_IDX]);
            }
            else {
                _led_ctrl(HAL_LED_CH_2, HAL_LED_LEVEL_OFF);
            }
            meas_set_data.led_ctrl_on[CH2_IDX] = ctrl;
            break;

        case MEAS_SET_CH_3:
            if (LED_CTRL_FORCE_ON == ctrl) {
                _led_ctrl(HAL_LED_CH_3, meas_set_data.led_on_level[CH3_IDX]);
            }
            else {
                _led_ctrl(HAL_LED_CH_3, HAL_LED_LEVEL_OFF);
            }
            meas_set_data.led_ctrl_on[CH3_IDX] = ctrl;
            break;

        case MEAS_SET_CH_ALL:
            if (LED_CTRL_FORCE_ON == ctrl) {
                _led_ctrl(HAL_LED_CH_1, meas_set_data.led_on_level[CH1_IDX]);
                _led_ctrl(HAL_LED_CH_2, meas_set_data.led_on_level[CH2_IDX]);
                _led_ctrl(HAL_LED_CH_3, meas_set_data.led_on_level[CH3_IDX]);
            }
            else {
                _led_ctrl(HAL_LED_CH_1, HAL_LED_LEVEL_OFF);
                _led_ctrl(HAL_LED_CH_2, HAL_LED_LEVEL_OFF);
                _led_ctrl(HAL_LED_CH_3, HAL_LED_LEVEL_OFF);
            }
            meas_set_data.led_ctrl_on[CH1_IDX] = ctrl;
            meas_set_data.led_ctrl_on[CH2_IDX] = ctrl;
            meas_set_data.led_ctrl_on[CH3_IDX] = ctrl;
            break;

        default:
            SYS_LOG_ERR("Invalid LED Control");
            return HAL_ERROR;
    }

    /* FRAM Write */
    SYS_VERIFY_SUCCESS(Hal_Fram_Write(FRAM_DEV_CTRL_LED_STATUS_ADDR, FRAM_DEV_CTRL_LED_STATUS_DATA_LEN, meas_set_data.led_ctrl_on));

    return HAL_OK;
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
    uint8_t read_data[FRAM_DATA_MAX_LEN] = { 0, };

    if (HAL_OK == Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data)) {
        if (read_data[FRAM_STABLE_TEMPERATURE_ADDR] == 0) {
            /* Set default */
            _meas_set_temp_ctrl_on(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_TEMP_CTRL_ON);
            Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);
            _meas_set_led_on_time_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_TIME_MS);
            Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);
            _meas_set_led_on_level(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_LEVEL);
            Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);
            _meas_set_adc_sample_cnt(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_SAMPLE_CNT);
            Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);
            _meas_set_adc_delay_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_DELAY_MS);
            Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);
            _meas_set_stable_temperature_degree(MEAS_SET_DEFAULT_STABLE_TEMPERATURE_DEGREE_X100);
            Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);
        }
        else {
            /* Set read data */
            _meas_set_temp_ctrl_on(MEAS_SET_CH_1, read_data[FRAM_TEMP_SETTING_ADDR]);
            _meas_set_temp_ctrl_on(MEAS_SET_CH_2, read_data[FRAM_TEMP_SETTING_ADDR + FRAM_TEMP_SETTING_SINGLE_DATA_LEN]);
            _meas_set_temp_ctrl_on(MEAS_SET_CH_3, read_data[FRAM_TEMP_SETTING_ADDR + FRAM_TEMP_SETTING_SINGLE_DATA_LEN * 2]);
            _meas_set_led_on_time_ms(MEAS_SET_CH_1, read_data[FRAM_LED_ON_TIME_ADDR]);
            _meas_set_led_on_time_ms(MEAS_SET_CH_2, read_data[FRAM_LED_ON_TIME_ADDR + FRAM_LED_ON_TIME_SINGLE_DATA_LEN]);
            _meas_set_led_on_time_ms(MEAS_SET_CH_3, read_data[FRAM_LED_ON_TIME_ADDR + FRAM_LED_ON_TIME_SINGLE_DATA_LEN * 2]);
            _meas_set_led_on_level(MEAS_SET_CH_1, read_data[FRAM_LED_ON_LEVEL_ADDR]);
            _meas_set_led_on_level(MEAS_SET_CH_2, read_data[FRAM_LED_ON_LEVEL_ADDR + FRAM_LED_ON_LEVEL_SINGLE_DATA_LEN]);
            _meas_set_led_on_level(MEAS_SET_CH_3, read_data[FRAM_LED_ON_LEVEL_ADDR + FRAM_LED_ON_LEVEL_SINGLE_DATA_LEN * 2]);
            _meas_set_adc_sample_cnt(MEAS_SET_CH_1, read_data[FRAM_ADC_SAMPLE_CNT_ADDR]);
            _meas_set_adc_sample_cnt(MEAS_SET_CH_2, read_data[FRAM_ADC_SAMPLE_CNT_ADDR + FRAM_ADC_SAMPLE_CNT_SINGLE_DATA_LEN]);
            _meas_set_adc_sample_cnt(MEAS_SET_CH_3, read_data[FRAM_ADC_SAMPLE_CNT_ADDR + FRAM_ADC_SAMPLE_CNT_SINGLE_DATA_LEN * 2]);
            _meas_set_adc_delay_ms(MEAS_SET_CH_1, read_data[FRAM_ADC_DELAY_MS_ADDR]);
            _meas_set_adc_delay_ms(MEAS_SET_CH_2, read_data[FRAM_ADC_DELAY_MS_ADDR + FRAM_ADC_DELAY_MS_SINGLE_DATA_LEN]);
            _meas_set_adc_delay_ms(MEAS_SET_CH_3, read_data[FRAM_ADC_DELAY_MS_ADDR + FRAM_ADC_DELAY_MS_SINGLE_DATA_LEN * 2]);
            _meas_set_stable_temperature_degree(read_data[FRAM_STABLE_TEMPERATURE_SINGLE_DATA_LEN]);
        }
    }
}

static void _meas_result_init(void) {
    memset(&meas_result_data, 0, sizeof(MeasResultData_t));
}

static HAL_StatusTypeDef _meas_op_start(uint8_t ch) {
    for (uint8_t ch_idx = 0; ch_idx < CH_NUM; ch_idx++) {
        meas_req_status_data.target_ch[ch_idx] = (ch == ch_idx) ? MEAS_TARGET_CH_ACTIVE : MEAS_TARGET_CH_DEACTIV;
    }

    /* LED Control */
    SYS_LOG_INFO("[MEAS] Start led delay: %d msec", meas_set_data.adc_delay_ms[ch]);
    SYS_VERIFY_SUCCESS(Hal_Led_Ctrl((HalLedCh_t )ch, meas_set_data.led_on_level[ch]));
//    App_Timer_Start(APP_TIMER_TYPE_LED_ON_TIME, meas_set_data.led_on_time[ch], _meas_task_led_timeout_cb);
    
    /* Start Measure Sequence */
    meas_task_context.meas_state = MEAS_STATE_LED_ON;
    _meas_task_req_cb();

    return HAL_OK;
}

static void _meas_task_led_timeout_cb(void) {
    /* Stop Measure Sequence */
    meas_task_context.meas_state = MEAS_STATE_SEND_RESULT;
    _meas_task_req_cb();
}

static void _meas_task_req_cb(void) {
    static MeasSetChVal_t ch;
    HalTempData_t temperature;
    int16_t monitor_pd, recv_pd;
    uint16_t temperature_tempdata;

    switch (meas_task_context.meas_state) {
        case MEAS_STATE_STANDBY:
            Hal_Pd_Stop();
            break;

        case MEAS_STATE_LED_ON:
            for (ch = 0; ch <= MEAS_SET_CH_3; ch++) {
                if (MEAS_TARGET_CH_ACTIVE == meas_req_status_data.target_ch[ch]) {
                    SYS_LOG_DEBUG("[MEAS] CH %d Selected", ch);
                    break;
                }
            }

            /* Read ADC */
            if (HAL_OK == Hal_Pd_Start()) {
                if (HAL_OK == Hal_Pd_Read()) {
                    sample_idx = 0;
                    meas_task_context.meas_state = MEAS_STATE_ADC_REQ;
                }
            }
            else {
                meas_task_context.meas_state = MEAS_STATE_ERROR;
            }
            break;

        case MEAS_STATE_ADC_REQ:
            if (HAL_OK == Hal_Pd_Read()) {
                meas_task_context.meas_state = MEAS_STATE_ADC_DONE;
            }
            else {
                meas_task_context.meas_state = MEAS_STATE_ERROR;

                /* Continue Measure Sequence */
                _meas_task_req_cb();
            }
            break;

        case MEAS_STATE_ADC_DONE:
            /* Calculate Average */
            Hal_Pd_GetMonitorData(ch, &monitor_pd_buff[ch][sample_idx]);
            Hal_Pd_GetRecvData(ch, &recv_pd_buff[ch][sample_idx]);
            Hal_Temp_GetData(&temperature);
#if(FEATURE_TEMPERATURE_DATA_ADC == FEATURE_TEMPERATURE_DATA_TYPE)
            temperature_buff[CH1_IDX][sample_idx] = temperature.adc[CH1_IDX];
            temperature_buff[CH2_IDX][sample_idx] = temperature.adc[CH2_IDX];
            temperature_buff[CH3_IDX][sample_idx] = temperature.adc[CH3_IDX];
#else
            temperature_buff[CH1_IDX][sample_idx] = (uint16_t) (temperature.degree[CH1_IDX] * MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE_SCALE);
            temperature_buff[CH2_IDX][sample_idx] = (uint16_t) (temperature.degree[CH2_IDX] * MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE_SCALE);
            temperature_buff[CH3_IDX][sample_idx] = (uint16_t) (temperature.degree[CH3_IDX] * MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE_SCALE);
#endif

            /* Time count */
            if (++sample_idx >= meas_set_data.adc_sample_cnt[ch]) {
                /* Next Sequence */
                meas_task_context.meas_state = MEAS_STATE_SEND_RESULT;
                _meas_task_req_cb();
            }
            else {
                meas_task_context.meas_state = MEAS_STATE_ADC_REQ;
                App_Timer_Start(APP_TIMER_TYPE_ADC_DELAY, meas_set_data.adc_delay_ms[ch], _meas_task_req_cb);
            }
            break;

        case MEAS_STATE_SEND_RESULT:
            SYS_LOG_INFO("[MEAS] LED Time Complete.");
            /* LED Off */
            (void) Hal_Led_Ctrl(HAL_LED_CH_1, HAL_LED_LEVEL_OFF);
            (void) Hal_Led_Ctrl(HAL_LED_CH_2, HAL_LED_LEVEL_OFF);
            (void) Hal_Led_Ctrl(HAL_LED_CH_3, HAL_LED_LEVEL_OFF);

            /* Save result */
            recv_pd = _calc_pd_avr(recv_pd_buff[ch], meas_set_data.adc_sample_cnt[ch]);
            monitor_pd = _calc_pd_avr(monitor_pd_buff[ch], meas_set_data.adc_sample_cnt[ch]);
            temperature_tempdata = ARRAY_AVERAGE(temperature_buff[ch], meas_set_data.adc_sample_cnt[ch]);
            meas_result_data.recv_pd_data[ch] = recv_pd;
            meas_result_data.monitor_pd_data[ch] = monitor_pd;
            meas_result_data.temperature_data[ch] = temperature_tempdata;
            SYS_LOG_INFO("[MEAS] ADC Result");
            SYS_LOG_INFO("- receive pd : %d", meas_result_data.recv_pd_data[ch]);
            SYS_LOG_INFO("- monitor pd : %d", meas_result_data.monitor_pd_data[ch]);
            SYS_LOG_INFO("- temperature: %d", meas_result_data.temperature_data[ch]);

            meas_task_context.meas_state = MEAS_STATE_STANDBY;
            SYS_LOG_INFO("[MEAS] ADC Read Complete. Send MMI Message");
            Task_MMI_SendMeasResult();
            break;

        case MEAS_STATE_ERROR:
            SYS_LOG_ERR("[MEAS] Error State");
            meas_task_context.meas_state = MEAS_STATE_STANDBY;
            break;
    }
}

static void _meas_set_temp_ctrl_on(MeasSetChVal_t ch, MeasSetTempCtrlVal_t val) {
    HalHeaterCh_t heater_ch;

    switch (ch) {
        case MEAS_SET_CH_1:
            meas_set_data.temp_ctrl_on[CH1_IDX] = val;
            heater_ch = HAL_HEATER_CH_1;
            break;

        case MEAS_SET_CH_2:
            meas_set_data.temp_ctrl_on[CH2_IDX] = val;
            heater_ch = HAL_HEATER_CH_2;
            break;

        case MEAS_SET_CH_3:
            meas_set_data.temp_ctrl_on[CH3_IDX] = val;
            heater_ch = HAL_HEATER_CH_3;
            break;

        case MEAS_SET_CH_ALL:
            meas_set_data.temp_ctrl_on[CH1_IDX] = val;
            meas_set_data.temp_ctrl_on[CH2_IDX] = val;
            meas_set_data.temp_ctrl_on[CH3_IDX] = val;
            heater_ch = HAL_HEATER_CH_ALL;
            break;
    }

    /* Temperature Control */
    switch (val) {
        case TEMP_CTRL_OFF:
            App_Timer_Stop(APP_TIMER_TYPE_HEATER_CTRL);
            Hal_Heater_Ctrl(heater_ch, HAL_HEATER_OFF);
            break;

        case TEMP_CTRL_AUTO_ON:
            /* Auto Control */
            App_Timer_Start(APP_TIMER_TYPE_HEATER_CTRL, MEAS_SET_STABLE_TEMPERATURE_CTRL_DUTY_MS, _heater_ctrl);
            break;

        case TEMP_CTRL_FORCE_ON:
            App_Timer_Stop(APP_TIMER_TYPE_HEATER_CTRL);
            Hal_Heater_Ctrl(heater_ch, HAL_HEATER_ON);
            break;
    }

    SYS_LOG_INFO("Temperature Control On/Off settings: %d, %d, %d", meas_set_data.temp_ctrl_on[CH1_IDX], meas_set_data.temp_ctrl_on[CH2_IDX], meas_set_data.temp_ctrl_on[CH3_IDX]);

    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Write(FRAM_TEMP_SETTING_ADDR, FRAM_TEMP_SETTING_DATA_LEN, (uint8_t *)meas_set_data.temp_ctrl_on));
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

    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Write(FRAM_LED_ON_TIME_ADDR, FRAM_LED_ON_TIME_DATA_LEN, (uint8_t *)meas_set_data.led_on_time));
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

    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Write(FRAM_LED_ON_LEVEL_ADDR, FRAM_LED_ON_LEVEL_DATA_LEN, (uint8_t *)meas_set_data.led_on_level));
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

    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Write(FRAM_ADC_SAMPLE_CNT_ADDR, FRAM_ADC_SAMPLE_CNT_DATA_LEN, (uint8_t *)meas_set_data.adc_sample_cnt));
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

    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Write(FRAM_ADC_DELAY_MS_ADDR, FRAM_ADC_DELAY_MS_DATA_LEN, (uint8_t *)meas_set_data.adc_delay_ms));
}

static void _meas_set_stable_temperature_degree(uint16_t val) {
    if (val > MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE_X100) {
        SYS_LOG_WARN("stable temperature settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE_X100;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }
    else if (val < MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE_X100) {
        SYS_LOG_WARN("stable temperature settings changed");
        SYS_LOG_WARN("Original: [%d]====>", val);
        val = MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE_X100;
        SYS_LOG_WARN("Changed : ====>[%d]", val);
    }

    meas_set_data.stable_temperature = val;

    SYS_LOG_INFO("Stable Temperature setting: %d ('C)", meas_set_data.stable_temperature);

    SYS_VERIFY_SUCCESS_VOID(Hal_Fram_Write(FRAM_STABLE_TEMPERATURE_ADDR, FRAM_STABLE_TEMPERATURE_DATA_LEN, (uint8_t *)&meas_set_data.stable_temperature));
}

static HAL_StatusTypeDef _meas_get_temperature_data(void) {
    HalTempData_t temp_data_buff;

    SYS_VERIFY_SUCCESS(Hal_Temp_GetData(&temp_data_buff));

#if(FEATURE_TEMPERATURE_DATA_ADC == FEATURE_TEMPERATURE_DATA_TYPE)
    meas_result_data.temperature_data[CH1_IDX] = temp_data_buff.adc[HAL_TEMP_CH_0];
    meas_result_data.temperature_data[CH2_IDX] = temp_data_buff.adc[HAL_TEMP_CH_1];
    meas_result_data.temperature_data[CH3_IDX] = temp_data_buff.adc[HAL_TEMP_CH_2];
#else
    meas_result_data.temperature_data[CH1_IDX] = temp_data_buff.degree[HAL_TEMP_CH_0];
    meas_result_data.temperature_data[CH2_IDX] = temp_data_buff.degree[HAL_TEMP_CH_1];
    meas_result_data.temperature_data[CH3_IDX] = temp_data_buff.degree[HAL_TEMP_CH_2];
#endif

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
            meas_result_data.recv_pd_data[ch_idx] = _calc_pd_avr(recv_pd_buff[ch_idx], meas_set_data.adc_sample_cnt[ch_idx]);
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
        meas_result_data.recv_pd_data[ch] = _calc_pd_avr(recv_pd_buff[ch], meas_set_data.adc_sample_cnt[ch]);
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
            meas_result_data.monitor_pd_data[ch_idx] = _calc_pd_avr(monitor_pd_buff[ch_idx], meas_set_data.adc_sample_cnt[ch_idx]);
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
        meas_result_data.monitor_pd_data[ch] = _calc_pd_avr(monitor_pd_buff[ch], meas_set_data.adc_sample_cnt[ch]);
    }
    
    return HAL_OK;
}

static void _heater_ctrl(void) {
    HalTempData_t temp_data;

    for (HalHeaterCh_t ch_idx = HAL_HEATER_CH_1; ch_idx < HAL_HEATER_CH_NUM; ch_idx++) {
        if (meas_set_data.temp_ctrl_on[ch_idx]) {
            SYS_VERIFY_SUCCESS_VOID(Hal_Temp_GetData(&temp_data));
            SYS_LOG_INFO("[CH %d] Current Temperature: %d", ch_idx + 1, (uint8_t )temp_data.degree[ch_idx]);

            if ((MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE_X100 >= temp_data.degree[ch_idx]) && (MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE_X100 <= temp_data.degree[ch_idx])) {
                if (temp_data.degree[ch_idx] * MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE_SCALE < MEAS_SET_DEFAULT_STABLE_TEMPERATURE_DEGREE_X100) {
                    Hal_Heater_Ctrl(ch_idx, HAL_HEATER_ON);
                }
                else {
                    Hal_Heater_Ctrl(ch_idx, HAL_HEATER_OFF);
                }
            }
            else {
                SYS_LOG_ERR("Invalid Temperature");
                Hal_Heater_Ctrl(ch_idx, HAL_HEATER_OFF);
            }
        }
    }
    App_Timer_Start(APP_TIMER_TYPE_HEATER_CTRL, MEAS_SET_STABLE_TEMPERATURE_CTRL_DUTY_MS, _heater_ctrl);
}

void _led_ctrl(HalLedCh_t ch, uint16_t set_data) {
    bool led_status = (set_data > 0) ? true : false;

    meas_req_status_data.ch_onoff_status[ch] = led_status;
    SYS_VERIFY_SUCCESS_VOID(Hal_Led_Ctrl(ch, set_data));
}

int16_t _calc_pd_avr(int16_t *p_buff, uint16_t sample_cnt) {
    int32_t sum = 0;
    int16_t result = 0;
    
    if (sample_cnt > 0) {
        for (uint16_t idx = 0; idx < sample_cnt; idx++) {
            sum += p_buff[idx];
        }
        result = (int16_t) (sum / sample_cnt);
        return result;
    }
    else {
        return 0;
    }
}
