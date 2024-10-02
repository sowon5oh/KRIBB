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
#include "task_meas.h"
#include "task_mmi.h"

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
    bool meas_done;
    meas_State_t meas_state;
} ledTaskContext_t;

/* Private define ------------------------------------------------------------*/
#define MEAS_CFG_DEFAULT_ADC_CH 			ADC_CH_0
#define MEAS_CFG_DEFAULT_WAIT_TIME_MS		400
#define MEAS_CFG_DEFAULT_MEAS_TIMEOUT_MS	5000
#define MEAS_CFG_DEFAULT_MEAS_NUM			100
#define MEAS_CFG_DEFAULT_SPS				ADC_SPS_32K

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef _meas_Operation(void);
static void _meas_state_changer(meas_State_t state);
static void _meas_task_init(void);
static void _meas_task_req_cb(float result);

/* Private variables ---------------------------------------------------------*/
ledTaskContext_t meas_task_context = {
    .meas_state = MEAS_STATE_STANDBY,
    .meas_done =
false, };


/* Public user code ----------------------------------------------------------*/
void Task_Meas_init(void) {
    _meas_task_init();
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


/* Private user code ---------------------------------------------------------*/
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

static void _meas_task_init(void) {
    meas_task_context.meas_state = MEAS_STATE_WAIT_START_CMD;
    meas_task_context.meas_done = false;
}

static void _meas_task_req_cb(float result) {
    meas_task_context.meas_done = true;
}

