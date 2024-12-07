/**
 ******************************************************************************
 * @file           : task_temp_ctrl.c
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
#include "task_temp_ctrl.h"
#include "task_meas.h"
#include "main.h"
#include "app_util.h"
#include "task_mmi.h"
#include "hal_drv_heater.h"
#include "hal_drv_temperature.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    bool task_op_state;
    uint16_t stable_temperature;
    float cur_temp[CH_NUM];
    MeasSetTempCtrlTypeVal_t temp_ctrl_type[CH_NUM];
} tempCtrlTaskContext_t;

/* Private define ------------------------------------------------------------*/
#define TEMP_CTRL_TASK_DUTY_MS 3000

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void _temp_ctrl_task_init(void);
static void _temp_ctrl_task_enable(bool enable);
static void _temp_ctrl_task_cb(void);

static void _set_temp_ctrl_type(MeasCh_t ch, MeasSetTempCtrlTypeVal_t temp_ctrl_type);
static void _set_stable_temp(float stable_temp);
static HAL_StatusTypeDef _fetch_temperature(void);
static void _temp_ctrl_task_cb(void);

/* Private variables ---------------------------------------------------------*/
static tempCtrlTaskContext_t temp_ctrl_task_context = {
    .task_op_state = false,
    .cur_temp[CH1_IDX] = 0.0f,
    .cur_temp[CH2_IDX] = 0.0f,
    .cur_temp[CH3_IDX] = 0.0f,
    .temp_ctrl_type[CH1_IDX] = TEMP_CTRL_OFF,
    .temp_ctrl_type[CH2_IDX] = TEMP_CTRL_OFF,
    .temp_ctrl_type[CH3_IDX] = TEMP_CTRL_OFF, };

/* Public user code ----------------------------------------------------------*/
void Task_TempCtrl_Init(void) {
    /* Sensor Init */
    SYS_VERIFY_SUCCESS_VOID(Hal_Heater_Init());
    SYS_VERIFY_SUCCESS_VOID(Hal_Temp_Init(&hadc1)); /* adc */
    SYS_VERIFY_SUCCESS_VOID(Hal_Temp_Start());
    
    _temp_ctrl_task_init();
}

HAL_StatusTypeDef Task_TempCtrl_Start(void) {
    _temp_ctrl_task_enable(true);
    SYS_LOG_INFO("[Temp-Ctrl] Task Start");
    
    return HAL_OK;
}

HAL_StatusTypeDef Task_TempCtrl_Stop(void) {
    _temp_ctrl_task_enable(false);
    SYS_LOG_INFO("[Temp-Ctrl] Task Stop");
    
    return HAL_OK;
}

HAL_StatusTypeDef Task_TempCtrl_SetStableTemp(float stable_temp) {
    _set_stable_temp(stable_temp);

    return HAL_OK;
}

HAL_StatusTypeDef Task_TempCtrl_GetCurTemp(MeasCh_t ch, float *p_temp) {
    SYS_VERIFY_PARAM_NOT_NULL(p_temp);

    *p_temp = temp_ctrl_task_context.cur_temp[ch];

    return HAL_OK;
}

HAL_StatusTypeDef Task_TempCtrl_SetCtrlType(MeasCh_t ch, MeasSetTempCtrlTypeVal_t type) {
    _set_temp_ctrl_type(ch, type);

    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
static void _temp_ctrl_task_init(void) {
    MeasSetData_t temp_set_data;

    Task_Meas_Get_Set(&temp_set_data);
    _set_stable_temp((float) temp_set_data.stable_temperature / MEAS_SET_TEMPERATURE_DEGREE_SCALE);
    _set_temp_ctrl_type(CH1_IDX, temp_set_data.temp_ctrl_type[CH1_IDX]);
    _set_temp_ctrl_type(CH2_IDX, temp_set_data.temp_ctrl_type[CH2_IDX]);
    _set_temp_ctrl_type(CH3_IDX, temp_set_data.temp_ctrl_type[CH3_IDX]);
}

static void _temp_ctrl_task_enable(bool enable) {
    if (enable) {
        temp_ctrl_task_context.task_op_state = true;
        
        /* Task Start */
        App_Task_Start(APP_TASK_TEMP_CTRL, TEMP_CTRL_TASK_DUTY_MS, _temp_ctrl_task_cb);
    }
    else {
        temp_ctrl_task_context.task_op_state = false;
        
        /* Task Stop */
        App_Task_Stop(APP_TASK_TEMP_CTRL);
    }
}

static void _temp_ctrl_task_cb(void) {
    _fetch_temperature();
    
    for (uint8_t ch_idx = CH1_IDX; ch_idx < CH_NUM; ch_idx++) {
        SYS_LOG_DEBUG("[CH %d] Current Temperature: %.2f", ch_idx + 1, temp_ctrl_task_context.cur_temp[ch_idx]);

        if (MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE < temp_ctrl_task_context.cur_temp[ch_idx]) {
            SYS_LOG_DEBUG("[CH %d] Temperature Over MAX Limit, %.2f", ch_idx + 1, temp_ctrl_task_context.cur_temp[ch_idx]);
            Hal_Heater_Ctrl(ch_idx, HAL_HEATER_OFF);
        }
        else if (MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE >= temp_ctrl_task_context.cur_temp[ch_idx]) {
            SYS_LOG_DEBUG("[CH %d] Temperature Below MIN Limit, %.2f", ch_idx + 1, temp_ctrl_task_context.cur_temp[ch_idx]);
            Hal_Heater_Ctrl(ch_idx, HAL_HEATER_ON);
        }
        else {
            SYS_LOG_DEBUG("[CH %d] Current Temp, %.2f", ch_idx + 1, temp_ctrl_task_context.cur_temp[ch_idx], temp_ctrl_task_context.cur_temp[ch_idx]);
            if (temp_ctrl_task_context.cur_temp[ch_idx] < temp_ctrl_task_context.stable_temperature) {
                Hal_Heater_Ctrl(ch_idx, HAL_HEATER_ON);
            }
            else {
                Hal_Heater_Ctrl(ch_idx, HAL_HEATER_OFF);
            }
        }
    }
}

static void _set_temp_ctrl_type(MeasCh_t ch, MeasSetTempCtrlTypeVal_t temp_ctrl_type) {
    /* Temperature Control */
    switch (temp_ctrl_type) {
        case TEMP_CTRL_OFF:
            Hal_Heater_Ctrl(ch, HAL_HEATER_OFF);
            break;

        case TEMP_CTRL_AUTO_ON:
            break;

        case TEMP_CTRL_FORCE_ON:
            Hal_Heater_Ctrl(ch, HAL_HEATER_ON);
            break;
    }
}

static void _set_stable_temp(float stable_temp) {
    if (stable_temp > MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE) {
        SYS_LOG_WARN("stable temperature settings changed");SYS_LOG_WARN("Original: [%.2f]====>", stable_temp);
        stable_temp = MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE;
        SYS_LOG_WARN("Changed : ====>[%.2f]", stable_temp);
    }
    else if (stable_temp < MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE) {
        SYS_LOG_WARN("stable temperature settings changed");SYS_LOG_WARN("Original: [%.2f]====>", stable_temp);
        stable_temp = MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE;
        SYS_LOG_WARN("Changed : ====>[%.2f]", stable_temp);
    }

    temp_ctrl_task_context.stable_temperature = stable_temp;

    SYS_LOG_INFO("Stable Temperature setting: %.2f ('C)", temp_ctrl_task_context.stable_temperature);
}

static HAL_StatusTypeDef _fetch_temperature(void) {
    HalTempData_t temp_data_buff;
    
    SYS_VERIFY_SUCCESS(Hal_Temp_GetData(&temp_data_buff));
    
#if(FEATURE_TEMPERATURE_DATA_ADC == FEATURE_TEMPERATURE_DATA_TYPE)
    temp_ctrl_task_context.cur_temp[CH1_IDX] = temp_data_buff.adc[HAL_TEMP_CH_0];
    temp_ctrl_task_context.cur_temp[CH2_IDX] = temp_data_buff.adc[HAL_TEMP_CH_1];
    temp_ctrl_task_context.cur_temp[CH3_IDX] = temp_data_buff.adc[HAL_TEMP_CH_2];
#else
#ifdef FEATURE_TEMPERATURE_DEGREE_OFFSET
    temp_ctrl_task_context.cur_temp[CH1_IDX] = temp_data_buff.degree[HAL_TEMP_CH_0] + FEATURE_TEMPERATURE_DEGREE_OFFSET;
    temp_ctrl_task_context.cur_temp[CH2_IDX] = temp_data_buff.degree[HAL_TEMP_CH_1] + FEATURE_TEMPERATURE_DEGREE_OFFSET;
    temp_ctrl_task_context.cur_temp[CH3_IDX] = temp_data_buff.degree[HAL_TEMP_CH_2] + FEATURE_TEMPERATURE_DEGREE_OFFSET;
#else
    temp_ctrl_task_context.cur_temp[CH1_IDX] = temp_data_buff.degree[HAL_TEMP_CH_0];
    temp_ctrl_task_context.cur_temp[CH2_IDX] = temp_data_buff.degree[HAL_TEMP_CH_1];
    temp_ctrl_task_context.cur_temp[CH3_IDX] = temp_data_buff.degree[HAL_TEMP_CH_2];
#endif
#endif

    SYS_LOG_INFO("Temperature: [CH 1] %.2f, [CH 2] %.2f, [CH 3] %.2f", temp_ctrl_task_context.cur_temp[CH1_IDX], temp_ctrl_task_context.cur_temp[CH2_IDX], temp_ctrl_task_context.cur_temp[CH3_IDX]);

    return HAL_OK;
}
