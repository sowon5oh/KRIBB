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
    MEAS_STATE_ADC_START,
    MEAS_STATE_ADC_WAIT,
    MEAS_STATE_ADC_DONE,
    MEAS_STATE_CH_CHANGE,
    MEAS_STATE_WAIT,
    MEAS_STATE_ERROR,
    MEAS_STATE_START = MEAS_STATE_LED_ON,
    MEAS_STATE_STOP = MEAS_STATE_ADC_START,
    MEAS_STATE_MAX = MEAS_STATE_ERROR,
} measState_t;

typedef struct {
    bool task_op_state;
    measState_t meas_state;

    MeasCtrlOpMode_t meas_op_mode;
    MeasCh_t meas_cur_ch;
    uint16_t meas_cnt;

    uint8_t sample_cnt;
} measTaskContext_t;

/* Private define ------------------------------------------------------------*/
#define MEAS_MSG_DEBUG_LOG           0
#define MEAS_TASK_DUTY_MS            10
#define MEAS_TASK_LED_STABLE_TIME_MS 100
#define MEAS_TASK_CH_STABLE_TIME_MS  5

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Init */
static void _meas_task_init(void);
static void _meas_task_enable(bool enable);
static void _meas_set_init(void);
static void _meas_result_init(void);
/* Timer Callback */
static void _meas_task_led_stable_cb(void);
static void _meas_task_adc_cb(void);
static void _meas_task_ch_stable_cb(void);
/* Task Callback */
static void _meas_task_cb(void);
/* Function */
static void _meas_set_temp_ctrl_mode(MeasSetChVal_t ch, MeasSetTempCtrlType_t val);
static void _meas_set_led_on_time_ms(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_led_on_level(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_sample_cnt(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_adc_delay_ms(MeasSetChVal_t ch, uint16_t val);
static void _meas_set_ch_test(MeasSetChVal_t ch, MeasSetChTest_t state);
static void _meas_set_stable_temperature_degree(uint16_t val);
static void _meas_set_temperature_offset_degree(MeasSetChVal_t ch, uint16_t val);
static void _meas_get_temperature_data(void);
static void _led_ctrl(MeasCh_t ch, uint16_t set_data);
int16_t _calc_pd_avr(int16_t *p_buff, uint16_t sample_cnt);

/* Private variables ---------------------------------------------------------*/
static measTaskContext_t meas_task_context = {
    .task_op_state = false,
    .meas_state = MEAS_STATE_START,
    .meas_op_mode = MEAS_OP_MODE_SINGLE,
    .meas_cur_ch = CH1_IDX,
    .meas_cnt = 0,
    .sample_cnt = 0, };
static MeasSetData_t meas_set_data;
static MeasResultData_t meas_result_data;
static MeasReqStatus_t meas_req_status_data;
static int16_t recv_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT] = {
    0, };
static int16_t monitor_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT] = {
    0, };

#if(CONFIG_FEATURE_SETTINGS_DEFAULT == 1)
static MeasSetData_t meas_set_default_data = {
    .temp_ctrl_mode[CH1_IDX] = MEAS_SET_DEFAULT_temp_ctrl_mode,
    .temp_ctrl_mode[CH2_IDX] = MEAS_SET_DEFAULT_temp_ctrl_mode,
    .temp_ctrl_mode[CH3_IDX] = MEAS_SET_DEFAULT_temp_ctrl_mode,
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
    .stable_temperature = MEAS_SET_DEFAULT_STABLE_TEMPERATURE_DEGREE,
    .temperature_offset[CH1_IDX] = MEAS_SET_DEFAULT_TEMPERATURE_OFFSET_DEGREE,
    .temperature_offset[CH2_IDX] = MEAS_SET_DEFAULT_TEMPERATURE_OFFSET_DEGREE,
    .temperature_offset[CH3_IDX] = MEAS_SET_DEFAULT_TEMPERATURE_OFFSET_DEGREE };
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
            MeasSetTempCtrlType_t temp_ctrl_val = (MeasSetTempCtrlType_t) p_set_val[0];
            _meas_set_temp_ctrl_mode(ch, temp_ctrl_val);
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

        case MEAS_SET_CAT_CH_TEST: {
            uint16_t set_val = p_set_val[0];
            _meas_set_ch_test(ch, set_val);
            break;
        }

        case MEAS_SET_CAT_TEMPERATURE_OFFSET: {
            uint16_t set_val = UINT8_2BYTE_ARRAY_TO_UINT16(p_set_val);
            _meas_set_temperature_offset_degree(ch, set_val);
            Hal_Fram_Write(FRAM_TEMPERATURE_OFFSET_ADDR_CH1 + (ch - 1) * 2, FRAM_TEMPERATURE_OFFSET_SINGLE_DATA_LEN, (uint8_t*) &set_val);
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

    /* ADC Stop */
    Hal_Pd_Start();
    SYS_LOG_INFO("[MEASURE START]"); //
    SYS_LOG_INFO("- LED ON TIME MS    : %3d / %3d / %3d", meas_set_data.led_on_time[CH1_IDX], meas_set_data.led_on_time[CH2_IDX], meas_set_data.led_on_time[CH3_IDX]); //
    SYS_LOG_INFO("- LED ON LEVEL      : %4d / %4d / %4d", meas_set_data.led_on_level[CH1_IDX], meas_set_data.led_on_level[CH2_IDX], meas_set_data.led_on_level[CH3_IDX]); //
    SYS_LOG_INFO("- ADC DELAY MS      : %4d / %4d / %4d", meas_set_data.adc_delay_ms[CH1_IDX], meas_set_data.adc_delay_ms[CH2_IDX], meas_set_data.adc_delay_ms[CH3_IDX]); //
    SYS_LOG_INFO("- ADC SAMPLE CNT    : %2d / %2d / %2d", meas_set_data.adc_sample_cnt[CH1_IDX], meas_set_data.adc_sample_cnt[CH2_IDX], meas_set_data.adc_sample_cnt[CH3_IDX]); //
    SYS_LOG_INFO("- STABLE TEMPERATURE: %d", meas_set_data.stable_temperature);

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Stop(void) {
    _meas_task_enable(false);

    /* ADC Stop */
    Hal_Pd_Stop();

    SYS_LOG_INFO("[MEASURE STOP]");

    return HAL_OK;
}

void Task_Meas_Req_ContinuosMode(void) {
    /* All Ch Mode Operation */
    meas_task_context.meas_op_mode = MEAS_OP_MODE_CONTINUOS;
    meas_task_context.meas_cur_ch = CH1_IDX;
    meas_task_context.meas_cnt = 0;
    /* Change to LED Auto Ctrl Mode */
    meas_set_data.led_ctrl_mode[CH1_IDX] = LED_CTRL_AUTO;
    meas_set_data.led_ctrl_mode[CH2_IDX] = LED_CTRL_AUTO;
    meas_set_data.led_ctrl_mode[CH3_IDX] = LED_CTRL_AUTO;

    SYS_LOG_INFO("[CONTINUOS MEASURE START]");
    _meas_task_enable(true);
}

void Task_Meas_Req_SingleMode(MeasSetChVal_t ch, uint16_t cnt) {
    /* Single Ch Mode Operation */
    meas_task_context.meas_op_mode = MEAS_OP_MODE_SINGLE;
    meas_task_context.meas_cnt = cnt;

    /* Change to LED Auto Ctrl Mode */
    meas_set_data.led_ctrl_mode[CH1_IDX] = LED_CTRL_AUTO;
    meas_set_data.led_ctrl_mode[CH2_IDX] = LED_CTRL_AUTO;
    meas_set_data.led_ctrl_mode[CH3_IDX] = LED_CTRL_AUTO;

    /* Set Ch */
    if (ch == MEAS_SET_CH_ALL) {
        meas_task_context.meas_cur_ch = CH1_IDX;
    }
    else {
        meas_task_context.meas_cur_ch = (MeasCh_t) (ch - 1);
    }

    SYS_LOG_INFO("[SINGLE MEASURE START, CH: %d]", ch);
    _meas_task_enable(true);
}

HAL_StatusTypeDef Task_Meas_Ctrl_OpMode(MeasCtrlOpMode_t op_mode) {
    SYS_VERIFY_TRUE(op_mode < MEAS_OP_MODE_MAX);

    _meas_task_enable(false);

    meas_task_context.meas_op_mode = op_mode;
    SYS_LOG_INFO("[MEASURE CTRL SET TO ]", (op_mode == MEAS_OP_MODE_SINGLE) ? "SINGLE" : "CONTINOUS");

    /* Not Used */

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_AllChResult(MeasResultData_t *p_data) {
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    /* Send Current Meas Data */
    _meas_get_temperature_data();
    memcpy(p_data, &meas_result_data, sizeof(MeasResultData_t));

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_SingleChResult(MeasSetChVal_t *p_ch, uint16_t *p_cnt, int16_t *p_recv, int16_t *p_temp) {
    SYS_VERIFY_PARAM_NOT_NULL(p_ch);
    SYS_VERIFY_PARAM_NOT_NULL(p_cnt);
    SYS_VERIFY_PARAM_NOT_NULL(p_recv);
    SYS_VERIFY_PARAM_NOT_NULL(p_temp);

    /* Send Current Meas Data */
    _meas_get_temperature_data();

    *p_ch = (MeasSetChVal_t) meas_task_context.meas_cur_ch + 1;
    *p_cnt = meas_task_context.meas_cnt;
    *p_recv = meas_result_data.recv_pd_data[meas_task_context.meas_cur_ch];
    *p_temp = meas_result_data.temperature_data[meas_task_context.meas_cur_ch];

    return HAL_OK;
}

HAL_StatusTypeDef Task_Meas_Get_Result(MeasResultCat_t result_cat, uint16_t *p_result_val) {
    SYS_VERIFY_TRUE(result_cat <= MEAS_RESULT_CAT_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_result_val);
    
    switch (result_cat) {
        case MEAS_RESULT_CAT_TEMPERATURE:
            _meas_get_temperature_data();
            memcpy(p_result_val, &meas_result_data.temperature_data, sizeof(uint16_t));
            return HAL_OK;

        case MEAS_RESULT_CAT_RECV_PD_ADC:
            memcpy(p_result_val, &meas_result_data.recv_pd_data, sizeof(uint16_t));
            return HAL_OK;

        case MEAS_RESULT_CAT_MONITOR_PD_ADC:
            memcpy(p_result_val, &meas_result_data.monitor_pd_data, sizeof(uint16_t));
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

HAL_StatusTypeDef Task_Meas_Ctrl_Led(MeasSetChVal_t ch, MeasCtrlLedType_t ctrl) {
    SYS_VERIFY_TRUE(ch <= MEAS_SET_CH_MAX);

    uint8_t levels[3] = {
        CH1_IDX,
        CH2_IDX,
        CH3_IDX };
    
    switch (ch) {
        case MEAS_SET_CH_1:
        case MEAS_SET_CH_2:
        case MEAS_SET_CH_3:
            if (ctrl == LED_CTRL_FORCE_ON) {
                _led_ctrl(HAL_LED_CH_1 + (ch - MEAS_SET_CH_1), meas_set_data.led_on_level[levels[ch - MEAS_SET_CH_1]]);
            }
            else if (ctrl == LED_CTRL_FORCE_OFF) {
                _led_ctrl(HAL_LED_CH_1 + (ch - MEAS_SET_CH_1), HAL_LED_LEVEL_OFF);
            }
            else if (ctrl == LED_CTRL_AUTO) {
                /* not control */
            }
            meas_set_data.led_ctrl_mode[levels[ch - MEAS_SET_CH_1]] = ctrl;
            break;

        case MEAS_SET_CH_ALL:
            for (uint8_t i = 0; i < 3; i++) {
                if (ctrl == LED_CTRL_FORCE_ON) {
                    _led_ctrl(HAL_LED_CH_1 + i, meas_set_data.led_on_level[i]);
                }
                else if (ctrl == LED_CTRL_FORCE_OFF) {
                    _led_ctrl(HAL_LED_CH_1 + i, HAL_LED_LEVEL_OFF);
                }
                else if (ctrl == LED_CTRL_AUTO) {
                    /* not control */
                }
                meas_set_data.led_ctrl_mode[i] = ctrl;
            }
            break;

        default:
            SYS_LOG_ERR("Invalid LED Control: ch=%d, ctrl=%d", ch, ctrl);
            return HAL_ERROR;
    }

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
        meas_task_context.sample_cnt = 0;

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
    uint16_t temp_temperature;
    uint8_t read_data[FRAM_DATA_MAX_LEN] = {
        0, };

    Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data);

#if(CONFIG_FEATURE_SETTINGS_DEFAULT == 1)
    /* Set default */
    _meas_set_temp_ctrl_mode(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_temp_ctrl_mode);
    _meas_set_led_on_time_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_TIME_MS);
    _meas_set_led_on_level(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_LED_ON_LEVEL);
    _meas_set_adc_sample_cnt(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_SAMPLE_CNT);
    _meas_set_adc_delay_ms(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_ADC_DELAY_MS);
    _meas_set_stable_temperature_degree(MEAS_SET_DEFAULT_STABLE_TEMPERATURE_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE);
    _meas_set_temperature_offset_degree(MEAS_SET_CH_ALL, MEAS_SET_DEFAULT_TEMPERATURE_OFFSET_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE);

    /* Save Fram */
    Hal_Fram_Write(FRAM_TEMP_SETTING_ADDR_CH1, FRAM_TEMP_SETTING_DATA_LEN, (uint8_t*) meas_set_default_data.temp_ctrl_mode);
    Hal_Fram_Write(FRAM_LED_ON_TIME_ADDR_CH1, FRAM_LED_ON_TIME_DATA_LEN, (uint8_t*) meas_set_default_data.led_on_time);
    Hal_Fram_Write(FRAM_LED_ON_LEVEL_ADDR_CH1, FRAM_LED_ON_LEVEL_DATA_LEN, (uint8_t*) meas_set_default_data.led_on_level);
    Hal_Fram_Write(FRAM_ADC_SAMPLE_CNT_ADDR_CH1, FRAM_ADC_SAMPLE_CNT_DATA_LEN, (uint8_t*) meas_set_default_data.adc_sample_cnt);
    Hal_Fram_Write(FRAM_ADC_DELAY_MS_ADDR_CH1, FRAM_ADC_DELAY_MS_DATA_LEN, (uint8_t*) meas_set_default_data.adc_delay_ms);

    temp_temperature = meas_set_default_data.stable_temperature * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
    Hal_Fram_Write(FRAM_STABLE_TEMPERATURE_ADDR, FRAM_STABLE_TEMPERATURE_DATA_LEN, (uint8_t*) &temp_temperature);

    temp_temperature = meas_set_default_data.temperature_offset[CH1_IDX] * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
    Hal_Fram_Write(FRAM_TEMPERATURE_OFFSET_ADDR_CH1, FRAM_TEMPERATURE_OFFSET_DATA_LEN, (uint8_t*) &temp_temperature);
    Hal_Fram_Write(FRAM_TEMPERATURE_OFFSET_ADDR_CH2, FRAM_TEMPERATURE_OFFSET_DATA_LEN, (uint8_t*) &temp_temperature);
    Hal_Fram_Write(FRAM_TEMPERATURE_OFFSET_ADDR_CH3, FRAM_TEMPERATURE_OFFSET_DATA_LEN, (uint8_t*) &temp_temperature);

    Hal_Fram_Read(FRAM_DATA_MIN_ADDR, FRAM_DATA_MAX_LEN, read_data); /* for check */
#else
    /* Set read data */
    _meas_set_temp_ctrl_mode(MEAS_SET_CH_1, read_data[FRAM_TEMP_SETTING_ADDR_CH1]);
    _meas_set_temp_ctrl_mode(MEAS_SET_CH_2, read_data[FRAM_TEMP_SETTING_ADDR_CH2]);
    _meas_set_temp_ctrl_mode(MEAS_SET_CH_3, read_data[FRAM_TEMP_SETTING_ADDR_CH3]);

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

    temp_temperature = (read_data[FRAM_STABLE_TEMPERATURE_ADDR + 1] << 8 | read_data[FRAM_STABLE_TEMPERATURE_ADDR]);
    _meas_set_stable_temperature_degree((float) temp_temperature);

    temp_temperature = (read_data[FRAM_TEMPERATURE_OFFSET_ADDR_CH1 + 1] << 8 | read_data[FRAM_TEMPERATURE_OFFSET_ADDR_CH1]);
    _meas_set_temperature_offset_degree(MEAS_SET_CH_1, (float) temp_temperature);
    temp_temperature = (read_data[FRAM_TEMPERATURE_OFFSET_ADDR_CH2 + 1] << 8 | read_data[FRAM_TEMPERATURE_OFFSET_ADDR_CH2]);
    _meas_set_temperature_offset_degree(MEAS_SET_CH_2, (float) temp_temperature);
    temp_temperature = (read_data[FRAM_TEMPERATURE_OFFSET_ADDR_CH3 + 1] << 8 | read_data[FRAM_TEMPERATURE_OFFSET_ADDR_CH3]);
    _meas_set_temperature_offset_degree(MEAS_SET_CH_3, (float) temp_temperature);

    /* Set other default data */
    meas_set_data.led_ctrl_mode[CH1_IDX] = MEAS_SET_DEFAULT_LED_CTRL_TYPE;
    meas_set_data.led_ctrl_mode[CH2_IDX] = MEAS_SET_DEFAULT_LED_CTRL_TYPE;
    meas_set_data.led_ctrl_mode[CH3_IDX] = MEAS_SET_DEFAULT_LED_CTRL_TYPE;
#endif

    SYS_LOG_INFO("Settings Done");
}

static void _meas_result_init(void) {
    memset(&meas_result_data, 0, sizeof(MeasResultData_t));
    memset(&recv_pd_buff, 0, sizeof(recv_pd_buff));
    memset(&monitor_pd_buff, 0, sizeof(monitor_pd_buff));
}

static void _meas_task_led_stable_cb(void) {
    meas_task_context.meas_state = MEAS_STATE_ADC_START;
}

static void _meas_task_adc_cb(void) {
    MeasCh_t cur_ch = meas_task_context.meas_cur_ch;

    Hal_Pd_GetRecvData(cur_ch, &recv_pd_buff[cur_ch][meas_task_context.sample_cnt]);
    Hal_Pd_GetMonitorData(cur_ch, &monitor_pd_buff[cur_ch][meas_task_context.sample_cnt]);

    if (++meas_task_context.sample_cnt >= meas_set_data.adc_sample_cnt[cur_ch]) {
        meas_result_data.recv_pd_data[cur_ch] = _calc_pd_avr(recv_pd_buff[cur_ch], meas_set_data.adc_sample_cnt[cur_ch]);
        meas_result_data.monitor_pd_data[cur_ch] = _calc_pd_avr(monitor_pd_buff[cur_ch], meas_set_data.adc_sample_cnt[cur_ch]);

        meas_task_context.meas_state = MEAS_STATE_ADC_DONE;
        meas_task_context.sample_cnt = 0;
    }
    else {
        meas_task_context.meas_state = MEAS_STATE_ADC_START;
    }
}

static void _meas_task_ch_stable_cb(void) {
    meas_task_context.meas_state = MEAS_STATE_START;
}

static void _meas_task_cb(void) {
    MeasCh_t cur_ch = meas_task_context.meas_cur_ch;

    if (meas_task_context.task_op_state == false) {
        SYS_LOG_WARN("Meas Operation Already Disabled");
        return;
    }

    switch (meas_task_context.meas_state) {
        case MEAS_STATE_LED_ON:
            //TODO Selected All Ch
            if (meas_req_status_data.target_ch[cur_ch] == MEAS_TARGET_CH_ACTIVE) {
                _led_ctrl(cur_ch, meas_set_data.led_on_level[cur_ch]);

                App_Timer_Start(APP_TIMER_ID_MEAS, MEAS_TASK_LED_STABLE_TIME_MS, false, _meas_task_led_stable_cb);
                meas_task_context.meas_state = MEAS_STATE_LED_STABLE;

#if(MEAS_MSG_DEBUG_LOG == 1)
                SYS_LOG_INFO("-----------------[ CH %d Measure ]-----------------", cur_ch + 1);
#endif
            }
            else {
                meas_task_context.meas_state = MEAS_STATE_ADC_DONE;
#if(MEAS_MSG_DEBUG_LOG == 1)
                SYS_LOG_INFO("-----------------[ CH %d Pass ]-----------------", cur_ch + 1);
#endif
            }
            break;

        case MEAS_STATE_LED_STABLE:
            /* Waiting Stable Time */
            break;

        case MEAS_STATE_ADC_START:
            /* Collecting Data */
            App_Timer_Start(APP_TIMER_ID_MEAS, meas_set_data.adc_delay_ms[cur_ch], false, _meas_task_adc_cb);
            meas_task_context.meas_state = MEAS_STATE_ADC_WAIT;
            break;

        case MEAS_STATE_ADC_WAIT:
            /* Wait for adc delay time */
            break;
            
        case MEAS_STATE_ADC_DONE:
            /* Calculate Average */
            Hal_Pd_GetRecvData(cur_ch, &meas_result_data.recv_pd_data[cur_ch]);
            Hal_Pd_GetMonitorData(cur_ch, &meas_result_data.monitor_pd_data[cur_ch]);

            /* Save result */
#if(MEAS_MSG_DEBUG_LOG == 1)
            SYS_LOG_INFO("- receive pd : %d", meas_result_data.recv_pd_data[cur_ch]);
            SYS_LOG_INFO("- monitor pd : %d", meas_result_data.monitor_pd_data[cur_ch]);
#endif
            
            /* LED Off */
            _led_ctrl(cur_ch, 0);

            switch (meas_task_context.meas_op_mode) {
                case MEAS_OP_MODE_CONTINUOS:
                    /* Change Channel */
                    if (++cur_ch >= CH_NUM) {
                        meas_task_context.meas_cur_ch = CH1_IDX;
                    }
                    else {
                        meas_task_context.meas_cur_ch = cur_ch;
                    }
                    meas_task_context.meas_state = MEAS_STATE_CH_CHANGE;

#if 1
                    /* Send MMI */
                    Task_MMI_SendMeasContinousResult();
#endif
                    break;

                case MEAS_OP_MODE_SINGLE:
                    /* 1 Frame */
                    meas_task_context.meas_state = MEAS_STATE_WAIT;

                    /* Meas End */
                    _meas_task_enable(false);

#if 1
                    /* Send MMI */
                    Task_MMI_SendMeasSigleResult();
#endif
                    break;

                default:
                    /* Not reached */
                    SYS_LOG_ERR("Invalid OP Mode");
                    break;
            }
            break;

        case MEAS_STATE_CH_CHANGE:
            if (HAL_OK == Hal_Pd_SetMonitorCh(cur_ch)) {
                /* wait for stable time */
                meas_task_context.meas_state = MEAS_STATE_WAIT;
                App_Timer_Start(APP_TIMER_ID_MEAS, MEAS_TASK_CH_STABLE_TIME_MS, false, _meas_task_ch_stable_cb);
            }
            else {
                meas_task_context.meas_state = MEAS_STATE_ERROR;
                SYS_LOG_ERR("Monitor Pd Ch Select Failed");
            }
            break;

        case MEAS_STATE_WAIT:
            /* Waiting Time */
            break;

        case MEAS_STATE_ERROR:
            /* Error State */
            return;
    }
}

static void _meas_set_temp_ctrl_mode(MeasSetChVal_t ch, MeasSetTempCtrlType_t val) {
    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.temp_ctrl_mode[CH1_IDX] = val;
        meas_set_data.temp_ctrl_mode[CH2_IDX] = val;
        meas_set_data.temp_ctrl_mode[CH3_IDX] = val;
        Task_TempCtrl_SetCtrlType(CH1_IDX, val);
        Task_TempCtrl_SetCtrlType(CH2_IDX, val);
        Task_TempCtrl_SetCtrlType(CH3_IDX, val);
    }
    else {
        meas_set_data.temp_ctrl_mode[ch - 1] = val;
    }

    SYS_LOG_INFO("Temperature Control On/Off settings: %d, %d, %d", meas_set_data.temp_ctrl_mode[CH1_IDX], meas_set_data.temp_ctrl_mode[CH2_IDX], meas_set_data.temp_ctrl_mode[CH3_IDX]);
}

static void _meas_set_led_on_time_ms(MeasSetChVal_t ch, uint16_t val) {
    uint16_t temp_val = val;

    if (val > MEAS_SET_MAX_LED_ON_TIME_MS) {
        SYS_LOG_WARN("LED On time settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_MAX_LED_ON_TIME_MS;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }

    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.led_on_time[CH1_IDX] = temp_val;
        meas_set_data.led_on_time[CH2_IDX] = temp_val;
        meas_set_data.led_on_time[CH3_IDX] = temp_val;
    }
    else {
        meas_set_data.led_on_time[ch - 1] = temp_val;
    }

    SYS_LOG_INFO("LED On time(ms) settings: %d, %d, %d", meas_set_data.led_on_time[CH1_IDX], meas_set_data.led_on_time[CH2_IDX], meas_set_data.led_on_time[CH3_IDX]);
}

static void _meas_set_led_on_level(MeasSetChVal_t ch, uint16_t val) {
    uint16_t temp_val = val;

    if (val > MEAS_SET_MAX_LED_ON_LEVEL) {
        SYS_LOG_WARN("LED On level settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_MAX_LED_ON_LEVEL;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }

    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.led_on_level[CH1_IDX] = temp_val;
        meas_set_data.led_on_level[CH2_IDX] = temp_val;
        meas_set_data.led_on_level[CH3_IDX] = temp_val;
    }
    else {
        meas_set_data.led_on_level[ch - 1] = temp_val;
    }

    SYS_LOG_INFO("LED On level(Digit) settings: %d, %d, %d", meas_set_data.led_on_level[CH1_IDX], meas_set_data.led_on_level[CH2_IDX], meas_set_data.led_on_level[CH3_IDX]);
}

static void _meas_set_adc_sample_cnt(MeasSetChVal_t ch, uint16_t val) {
    uint16_t temp_val = val;

    if (val > MEAS_SET_MAX_ADC_SAMPLE_CNT) {
        SYS_LOG_WARN("ADC sample count settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_MAX_ADC_SAMPLE_CNT;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }

    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.adc_sample_cnt[CH1_IDX] = temp_val;
        meas_set_data.adc_sample_cnt[CH2_IDX] = temp_val;
        meas_set_data.adc_sample_cnt[CH3_IDX] = temp_val;
    }
    else {
        meas_set_data.adc_sample_cnt[ch - 1] = temp_val;
    }

    SYS_LOG_INFO("ADC Sample count settings: %d, %d, %d", meas_set_data.adc_sample_cnt[CH1_IDX], meas_set_data.adc_sample_cnt[CH2_IDX], meas_set_data.adc_sample_cnt[CH3_IDX]);
}

static void _meas_set_adc_delay_ms(MeasSetChVal_t ch, uint16_t val) {
    uint16_t temp_val = val;

    if (val > MEAS_SET_MAX_ADC_DELAY_MS) {
        SYS_LOG_WARN("ADC delay time settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_MAX_ADC_DELAY_MS;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }

    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.adc_delay_ms[CH1_IDX] = temp_val;
        meas_set_data.adc_delay_ms[CH2_IDX] = temp_val;
        meas_set_data.adc_delay_ms[CH3_IDX] = temp_val;
    }
    else {
        meas_set_data.adc_delay_ms[ch - 1] = temp_val;
    }

    SYS_LOG_INFO("ADC Delay time(ms) settings: %d, %d, %d", meas_set_data.adc_delay_ms[CH1_IDX], meas_set_data.adc_delay_ms[CH2_IDX], meas_set_data.adc_delay_ms[CH3_IDX]);
}

static void _meas_set_stable_temperature_degree(uint16_t val) {
    uint16_t temp_val = val;

    if (val > (MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE)) {
        SYS_LOG_WARN("stable temperature settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }
    else if (val < ( MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE)) {
        SYS_LOG_WARN("stable temperature settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }

    meas_set_data.stable_temperature = temp_val;
    Task_TempCtrl_SetStableTemp((float) temp_val / MEAS_SET_TEMPERATURE_DEGREE_SCALE);

    SYS_LOG_INFO("Stable Temperature setting: %d(/100) 'C)", meas_set_data.stable_temperature);
}

static void _meas_set_ch_test(MeasSetChVal_t ch, MeasSetChTest_t state) {
    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.ch_test[CH1_IDX] = state;
        meas_set_data.ch_test[CH2_IDX] = state;
        meas_set_data.ch_test[CH3_IDX] = state;
    }
    else {
        meas_set_data.ch_test[ch - 1] = state;
    }

    if (state == CH_TEST_ON) {
        /* LED Forced On */
        meas_set_data.led_ctrl_mode[ch - 1] = LED_CTRL_FORCE_ON;
    }
    else if (state) {
        /* LED Auto */
        meas_set_data.led_ctrl_mode[ch - 1] = LED_CTRL_AUTO;
    }

    Task_MMI_SendMonitorPdResult(ch);
    SYS_LOG_INFO("CH Test Settings: %d, %d, %d (/100)'C)", meas_set_data.ch_test[CH1_IDX], meas_set_data.ch_test[CH2_IDX], meas_set_data.ch_test[CH3_IDX]);
}

static void _meas_set_temperature_offset_degree(MeasSetChVal_t ch, uint16_t val) {
    uint16_t temp_val = val;

    if (val > (MEAS_SET_TEMPERATURE_OFFSET_MAX_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE)) {
        SYS_LOG_WARN("temperature offset settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_TEMPERATURE_OFFSET_MAX_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }
    else if (val < (MEAS_SET_TEMPERATURE_OFFSET_MIN_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE)) {
        SYS_LOG_WARN("temperature offset settings changed");SYS_LOG_WARN("Original: [%d]====>", val);
        temp_val = MEAS_SET_TEMPERATURE_OFFSET_MIN_DEGREE * MEAS_SET_TEMPERATURE_DEGREE_SCALE;
        SYS_LOG_WARN("Changed : ====>[%d]", temp_val);
    }

    if (ch == MEAS_SET_CH_ALL) {
        meas_set_data.temperature_offset[CH1_IDX] = temp_val;
        meas_set_data.temperature_offset[CH2_IDX] = temp_val;
        meas_set_data.temperature_offset[CH3_IDX] = temp_val;
        Task_TempCtrl_SetTempOffset(CH1_IDX, (float) temp_val / MEAS_SET_TEMPERATURE_DEGREE_SCALE);
        Task_TempCtrl_SetTempOffset(CH2_IDX, (float) temp_val / MEAS_SET_TEMPERATURE_DEGREE_SCALE);
        Task_TempCtrl_SetTempOffset(CH3_IDX, (float) temp_val / MEAS_SET_TEMPERATURE_DEGREE_SCALE);
    }
    else {
        meas_set_data.temperature_offset[ch - 1] = temp_val;
        Task_TempCtrl_SetTempOffset(ch - 1, (float) temp_val / MEAS_SET_TEMPERATURE_DEGREE_SCALE);
    }

    SYS_LOG_INFO("Temperature Offset setting: %d, %d, %d (/100)'C)", meas_set_data.temperature_offset[CH1_IDX], meas_set_data.temperature_offset[CH2_IDX], meas_set_data
        .temperature_offset[CH3_IDX]);
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
