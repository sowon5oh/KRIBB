/**
 ******************************************************************************
 * @file           : task_fsm.c
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
#include "task_fsm.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    TASK_FSM_STATE_IDLE = 0,
    TASK_FSM_STATE_RUN,
    TASK_FSM_STATE_MAX
} fsmTaskState_t;

typedef struct {
    uint8_t id;
    char name[10];
} fsmTaskStateName_t;

typedef struct {
    uint16_t time_cnt;
    fsmTaskState_t task_state;
} fsmTaskContext_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void _fsm_task_state_hdl(fsmTaskState_t state);
void _fsm_task_init(void);

/* Private variables ---------------------------------------------------------*/
fsmTaskContext_t fsm_task_context = { .time_cnt = 0, .task_state = TASK_FSM_STATE_IDLE };
fsmTaskStateName_t fsm_state_info[TASK_FSM_STATE_MAX] = { { TASK_FSM_STATE_IDLE, "IDLE" }, { TASK_FSM_STATE_RUN, "RUN" }, };

/* Public user code ----------------------------------------------------------*/
void Task_Fsm_Init(void) {
    _fsm_task_init();
    //TODO
}

void Task_Fsm_Start(void) {
    if (fsm_task_context.task_state != TASK_FSM_STATE_RUN) {
        //TODO

        _fsm_task_state_hdl(TASK_FSM_STATE_RUN);
    }
}

void Task_Fsm_Stop(void) {
    if (fsm_task_context.task_state != TASK_FSM_STATE_IDLE) {
        //TODO

        _fsm_task_state_hdl(TASK_FSM_STATE_IDLE);
    }

    _fsm_task_init();
}

void Task_Fsm_Process(void) {
    switch (fsm_task_context.task_state) {
        case TASK_FSM_STATE_IDLE:
            break;

        case TASK_FSM_STATE_RUN:
            /* Timer Count */
            //TODO
            break;

        default:
            break;
    }
}

/* Private user code ---------------------------------------------------------*/
void _fsm_task_state_hdl(fsmTaskState_t state) {
    if (fsm_task_context.task_state != state) {
        LogInfo("**********************************");
        LogInfo("[TASK STATE] %s ======> %s", fsm_state_info[fsm_task_context.task_state].name, fsm_state_info[state].name);
        LogInfo("**********************************");
        fsm_task_context.task_state = state;
    }
}

void _fsm_task_init(void) {
    fsm_task_context.time_cnt = 0;
    fsm_task_context.task_state = TASK_FSM_STATE_IDLE;
}
