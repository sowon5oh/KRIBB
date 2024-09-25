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
typedef struct {
    uint8_t id;
    char name[5];
} FsmStateName_t;

typedef HAL_StatusTypeDef (*fsmActionFunc_t)(void);
typedef struct {
    FsmEvent_t event;
    FsmState_t cur_state;
    fsmActionFunc_t action_func;
    FsmState_t next_state;
} fsmTable_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void _fsm_hdl(FsmEvent_t event);
void _fsm_task_init(void);
HAL_StatusTypeDef _fsm_proc_meas_start(void);
HAL_StatusTypeDef _fsm_proc_meas_pause(void);
HAL_StatusTypeDef _fsm_proc_meas_done(void);
HAL_StatusTypeDef _fsm_proc_sleep(void);

/* Private variables ---------------------------------------------------------*/
static uint16_t fsm_polling_sec;
static FsmState_t fsm_cur_state;
static FsmStateName_t fsm_state_info[TASK_FSM_STATE_MAX] = {
    {
        TASK_FSM_STATE_IDLE,
        "IDLE" },
    {
        TASK_FSM_STATE_READY,
        "READY" },
    {
        TASK_FSM_STATE_MEAS,
        "MEAS" } };
static uint8_t fsm_evt_num_max;
static fsmTable_t fsm_table[] = {
    {
        TASK_FSM_EVENT_MEAS_REQ,
        TASK_FSM_STATE_IDLE,
        _fsm_proc_meas_start,
        TASK_FSM_STATE_MEAS },
    {
        TASK_FSM_EVENT_MEAS_REQ,
        TASK_FSM_STATE_READY,
        _fsm_proc_meas_start,
        TASK_FSM_STATE_MEAS },
    {
        TASK_FSM_EVENT_MEAS_DONE,
        TASK_FSM_STATE_MEAS,
        _fsm_proc_meas_done,
        TASK_FSM_STATE_READY },
    {
        TASK_FSM_EVENT_TIMEOUT,
        TASK_FSM_STATE_READY,
        _fsm_proc_sleep,
        TASK_FSM_STATE_IDLE } };

/* Public user code ----------------------------------------------------------*/
void Task_Fsm_Init(void) {
    _fsm_task_init();
}

void Task_Fsm_Start(void) {
    _fsm_hdl(TASK_FSM_STATE_READY);
}

void Task_Fsm_Stop(void) {
    _fsm_hdl(TASK_FSM_STATE_IDLE);
    _fsm_task_init();
}

void Task_Fsm_Process(void) {
    fsm_polling_sec += 1;
    
    //TODO
}

void Task_Fsm_SendEvent(FsmEvent_t event) {
    _fsm_hdl(event);
}
/* Private user code ---------------------------------------------------------*/
/**
 * @brief FSM handler function
 *
 * Handles the state transitions based on the provided event.
 * It checks the FSM table to find a matching event and current state,
 * and then executes the corresponding action function if available.
 *
 * @param event The event that triggers the state transition.
 */
void _fsm_hdl(FsmEvent_t event) {
    FsmState_t fsm_next_state = fsm_cur_state; /**< The next state to transition to if the event matches. */
    fsmActionFunc_t action_func = NULL; /**< The action function to be executed for the transition. */
    bool flag = false; /**< Flag to indicate if a matching state transition was found. */

    // Check if the event is within the valid range of FSM events.
    SYS_VERIFY_TRUE_VOID(event < TASK_FSM_EVENT_MAX);

    // Iterate through the FSM table to find a matching event and current state.
    for (uint8_t index = 0; index < fsm_evt_num_max; index++) {
        if ((event == fsm_table[index].event) && (fsm_cur_state == fsm_table[index].cur_state)) {
            flag = true;
            fsm_next_state = fsm_table[index].next_state;
            action_func = fsm_table[index].action_func;

            LogInfo("FSM Event: %d", event);
            break;
        }
    }

    // If a matching transition is found, proceed to state transition.
    if (flag == true) {
        if (action_func == NULL) {
            LogInfo("[FSM] %s ===[ No Action ]===> %s", fsm_state_info[fsm_cur_state].name, fsm_state_info[fsm_next_state].name);
            fsm_cur_state = fsm_next_state;
        }
        else {
            if (action_func() == HAL_OK) {
                LogInfo("[FSM] %s ===[ Action Completed ]===> %s", fsm_state_info[fsm_cur_state].name, fsm_state_info[fsm_next_state].name);
                fsm_cur_state = fsm_next_state;
            }
            else {
                LogError("[FSM] Action Failed");
            }
        }
    }
}

void _fsm_task_init(void) {
    /* init state machine */
    fsm_cur_state = TASK_FSM_STATE_IDLE;
    fsm_evt_num_max = sizeof(fsm_table) / sizeof(fsmTable_t);

    /* init polling timer */
    fsm_polling_sec = 0;
}

HAL_StatusTypeDef _fsm_proc_meas_start(void) {
    //TODO
    return HAL_OK;
}

HAL_StatusTypeDef _fsm_proc_meas_pause(void) {
    //TODO
    return HAL_OK;
}

HAL_StatusTypeDef _fsm_proc_meas_done(void) {
    //TODO
    return HAL_OK;
}

HAL_StatusTypeDef _fsm_proc_sleep(void) {
    fsm_polling_sec = 0;
    //TODO
    return HAL_OK;
}
