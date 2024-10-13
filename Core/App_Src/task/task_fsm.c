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
#include "hal_drv_heater.h"
#include "hal_drv_led.h"
#include "hal_drv_pd.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint8_t id;
    char name[10];
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
static void _fsm_task_init(void);
static void _fsm_send_event(FsmEvent_t event);
static void _fsm_hdl(FsmEvent_t event);
static HAL_StatusTypeDef _fsm_proc_test(void);
static HAL_StatusTypeDef _fsm_proc_sleep(void);
static HAL_StatusTypeDef _fsm_test_stop(void);

/* Private variables ---------------------------------------------------------*/
static uint16_t fsm_polling_sec;
static FsmState_t fsm_cur_state;
static FsmStateName_t fsm_state_info[TASK_FSM_STATE_NUM] = {
    {
        TASK_FSM_STATE_IDLE,
        "IDLE" },
    {
        TASK_FSM_STATE_READY,
        "READY" },
    {
        TASK_FSM_STATE_TEST,
        "MEAS" } };
static uint8_t fsm_evt_num_max;
static fsmTable_t fsm_table[] = {
    {
        TASK_FSM_EVENT_INIT_DONE,
        TASK_FSM_STATE_IDLE,
        NULL,
        TASK_FSM_STATE_READY },
    {
        TASK_FSM_EVENT_TEST_REQ,
        TASK_FSM_STATE_READY,
        NULL,
        TASK_FSM_STATE_TEST },
    {
        TASK_FSM_EVENT_TEST_DONE,
        TASK_FSM_STATE_TEST,
        _fsm_test_stop,
        TASK_FSM_STATE_READY },
    {
        TASK_FSM_EVENT_TIMEOUT,
        TASK_FSM_STATE_READY,
        NULL,
        TASK_FSM_STATE_IDLE } };
static FsmTaskTestType_t fsm_task_cur_test;
bool fsm_task_test_result = false;

/* Public user code ----------------------------------------------------------*/
void Task_Fsm_Init(void) {
    _fsm_task_init();
    _fsm_send_event(TASK_FSM_EVENT_INIT_DONE);
}

void Task_Fsm_Process(void) {
    switch (fsm_cur_state) {
        case TASK_FSM_STATE_IDLE:
        default:
            /* Unreachable case, should not occur under normal conditions */
            SYS_LOG_ERR("Unreachable state");
            break;

        case TASK_FSM_STATE_READY:
            _fsm_proc_sleep();
            break;

        case TASK_FSM_STATE_TEST:
            _fsm_proc_test();
            break;
    }
}

void Task_Fsm_SendEvent(FsmEvent_t event) {
    _fsm_send_event(event);
}

/* For Test code */
void Task_Fsm_StartTest(FsmTaskTestType_t test) {
    fsm_task_cur_test = test;
    SYS_LOG_TEST("%d Test Start", fsm_task_cur_test);
    _fsm_send_event(TASK_FSM_EVENT_TEST_REQ);
}

/* Private user code ---------------------------------------------------------*/
static void _fsm_send_event(FsmEvent_t event) {
    _fsm_hdl(event);
}

/**
 * @brief FSM handler function
 *
 * Handles the state transitions based on the provided event.
 * It checks the FSM table to find a matching event and current state,
 * and then executes the corresponding action function if available.
 *
 * @param event The event that triggers the state transition.
 */
static void _fsm_hdl(FsmEvent_t event) {
    FsmState_t fsm_next_state = fsm_cur_state; /**< The next state to transition to if the event matches. */
    fsmActionFunc_t action_func = NULL; /**< The action function to be executed for the transition. */
    bool flag = false; /**< Flag to indicate if a matching state transition was found. */
    
    /* Check if the event is within the valid range of FSM events. */
    SYS_VERIFY_TRUE_VOID(event < TASK_FSM_EVENT_MAX);
    
    /* Iterate through the FSM table to find a matching event and current state. */
    for (uint8_t index = 0; index < fsm_evt_num_max; index++) {
        if ((event == fsm_table[index].event) && (fsm_cur_state == fsm_table[index].cur_state)) {
            flag = true;
            fsm_next_state = fsm_table[index].next_state;
            action_func = fsm_table[index].action_func;

            SYS_LOG_INFO("FSM Event: %d", event);
            break;
        }
    }
    
    /* If a matching transition is found, proceed to state transition. */
    if (flag == true) {
        if (action_func == NULL) {
            SYS_LOG_INFO("[FSM] %s ===[ No Action ]===> %s", fsm_state_info[fsm_cur_state].name, fsm_state_info[fsm_next_state].name);
            fsm_cur_state = fsm_next_state;
        }
        else {
            if (action_func() == HAL_OK) {
                SYS_LOG_INFO("[FSM] %s ===[ Action Completed ]===> %s", fsm_state_info[fsm_cur_state].name, fsm_state_info[fsm_next_state].name);
                fsm_cur_state = fsm_next_state;
            }
            else {
                SYS_LOG_ERR("[FSM] Action Failed");
            }
        }
    }
}

static void _fsm_task_init(void) {
    /* init state machine */
    fsm_cur_state = TASK_FSM_STATE_IDLE;
    fsm_evt_num_max = sizeof(fsm_table) / sizeof(fsmTable_t);
    
    /* init polling timer */
    fsm_polling_sec = 0;
}

static HAL_StatusTypeDef _fsm_proc_sleep(void) {
    /* Do nothing */

    return HAL_OK;
}

static HAL_StatusTypeDef _fsm_proc_test(void) {
    switch (fsm_task_cur_test) {
        case FSM_TEST_MMI_DEVICE_CTRL_LED:
            //TODO
            break;

        default:
            SYS_LOG_ERR("Please Define the test");
            break;
    }

    _fsm_test_stop();

    return HAL_OK;
}

static HAL_StatusTypeDef _fsm_test_stop(void) {
    _fsm_send_event(TASK_FSM_EVENT_TEST_DONE);

    SYS_LOG_TEST("%d Test Stop. Result: %s", ((fsm_task_test_result)? "SUCCESS":"FAIL"));
    return HAL_OK;
}
