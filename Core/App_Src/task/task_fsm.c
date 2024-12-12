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
#include "hal_drv_temperature.h"
#include "hal_drv_fram.h"

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
static void _perform_mmi_test(const char *test_str);
static void _string_to_hex_array(const char *str, uint8_t *hex_array, size_t hex_array_len);

/* Private variables ---------------------------------------------------------*/
static uint16_t fsm_polling_sec;
static FsmState_t fsm_cur_state;
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
        _fsm_proc_test,
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
static FsmTaskTestMode_t fsm_test_mode = FSM_TEST_MODE_SINGLE;

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
void Task_Fsm_StartTest(FsmTaskTestType_t test, FsmTaskTestMode_t mode) {
    fsm_task_cur_test = test;
    fsm_test_mode = mode;

    SYS_LOG_INFO("%d Test Start", fsm_task_cur_test);
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

            SYS_LOG_INFO("[FSM] Event: %d", event);
            break;
        }
    }
    
    /* If a matching transition is found, proceed to state transition. */
    if (flag == true) {
        if (action_func == NULL) {
            fsm_cur_state = fsm_next_state;
        }
        else {
            if (action_func() == HAL_OK) {
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
        /***********************************************************************
         * Device Test
         ***********************************************************************/
        case FSM_TEST_DEVICE_LED_ONOFF:
            SYS_LOG_INFO("LED CH 1 ON");
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_1, HAL_LED_LEVEL_TEST));
            HAL_Delay(1000);
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_1, HAL_LED_LEVEL_OFF));
            SYS_LOG_INFO("LED CH 2 ON");
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_2, HAL_LED_LEVEL_TEST));
            HAL_Delay(1000);
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_2, HAL_LED_LEVEL_OFF));
            SYS_LOG_INFO("LED CH 3 ON");
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_3, HAL_LED_LEVEL_TEST));
            HAL_Delay(1000);
            SYS_VERIFY_SUCCESS(Hal_Led_Ctrl(HAL_LED_CH_3, HAL_LED_LEVEL_OFF));
            break;

        case FSM_TEST_DEVICE_HEATER_ONOFF:
            SYS_VERIFY_SUCCESS(Hal_Heater_Ctrl(HAL_HEATER_CH_1, HAL_HEATER_ON));
            SYS_VERIFY_SUCCESS(Hal_Heater_Ctrl(HAL_HEATER_CH_2, HAL_HEATER_ON));
            HAL_Delay(5000);
            SYS_VERIFY_SUCCESS(Hal_Heater_Ctrl(HAL_HEATER_CH_1, HAL_HEATER_OFF));
            SYS_VERIFY_SUCCESS(Hal_Heater_Ctrl(HAL_HEATER_CH_2, HAL_HEATER_OFF));
            break;

        case FSM_TEST_DEVICE_READ_TEMP:
            SYS_VERIFY_SUCCESS(Hal_Temp_Start());
            HAL_Delay(500);
//            HalTempData_t temp;
//            SYS_VERIFY_SUCCESS(Hal_Temp_GetData(&temp));
//            SYS_LOG_INFO("Temperature Read:");
//            SYS_LOG_INFO("[CH 0] ADC: %d, Degree: %d.%d", temp.adc[HAL_TEMP_CH_0], (int8_t )temp.degree[HAL_TEMP_CH_0], abs((int8_t )(temp.degree[HAL_TEMP_CH_0] * 100) % 100));
//            SYS_LOG_INFO("[CH 1] ADC: %d, Degree: %d.%d", temp.adc[HAL_TEMP_CH_1], (int8_t )temp.degree[HAL_TEMP_CH_1], abs((int8_t )(temp.degree[HAL_TEMP_CH_1] * 100) % 100));
//            SYS_LOG_INFO("[CH 2] ADC: %d, Degree: %d.%d", temp.adc[HAL_TEMP_CH_2], (int8_t )temp.degree[HAL_TEMP_CH_2], abs((int8_t )(temp.degree[HAL_TEMP_CH_2] * 100) % 100));

            uint16_t temp_data[3];
            Task_Meas_Get_Result(MEAS_RESULT_CAT_TEMPERATURE, MEAS_SET_CH_ALL, temp_data);
            break;

            /***********************************************************************
             * Measure Test
             ***********************************************************************/

            /***********************************************************************
             * MMI Message Test
             ***********************************************************************/
        case FSM_TEST_MMI_WHO_AM_I_DEV_INFO:
            _perform_mmi_test(MMI_TEST_STR_WHO_AM_I_DEV_INFO);
            break;

        case FSM_TEST_MMI_WHO_AM_I:
            _perform_mmi_test(MMI_TEST_STR_WHO_AM_I);
            break;

        case FSM_TEST_MMI_DEV_INFO:
            _perform_mmi_test(MMI_TEST_STR_DEV_INFO);
            break;

        case FSM_TEST_MMI_SET_TEMP_ONOFF:
            _perform_mmi_test(MMI_TEST_STR_SET_TEMP_ONOFF);
            break;

        case FSM_TEST_MMI_SET_LED_ON_TIME:
            _perform_mmi_test(MMI_TEST_STR_SET_LED_ON_TIME);
            break;

        case FSM_TEST_MMI_SET_LED_ON_LEVEL:
            _perform_mmi_test(MMI_TEST_STR_SET_LED_ON_LEVEL);
            break;

        case FSM_TEST_MMI_SET_ADC_SAMPLE_CNT:
            _perform_mmi_test(MMI_TEST_STR_SET_ADC_SAMPLE_CNT);
            break;

        case FSM_TEST_MMI_SET_ADC_DELAY_MS:
            _perform_mmi_test(MMI_TEST_STR_SET_ADC_DELAY_MS);
            break;

        case FSM_TEST_MMI_SET_STABLE_TEMPERATURE:
            _perform_mmi_test(MMI_TEST_STR_SET_STABLE_TEMPERATURE);
            break;
            
        case FSM_TEST_MMI_SET_REQ_ALL_SETTINGS:
            _perform_mmi_test(MMI_TEST_STR_SET_REQ_ALL_SETTINGS);
            break;

        case FSM_TEST_MMI_REQ_MEASURE:
            _perform_mmi_test(MMI_TEST_STR_REQ_ALL_DATA);
            break;

        case FSM_TEST_MMI_REQ_TEMPERATURE:
            _perform_mmi_test(MMI_TEST_STR_REQ_TEMPERATURE);
            break;
            
        case FSM_TEST_MMI_REQ_RECV_PD:
            _perform_mmi_test(MMI_TEST_STR_REQ_RECV_PD);
            break;
            
        case FSM_TEST_MMI_REQ_MONITOR_PD:
            _perform_mmi_test(MMI_TEST_STR_REQ_MONITOR_PD);
            break;
            
        case FSM_TEST_MMI_DEV_STATUS_REQ:
            _perform_mmi_test(MMI_TEST_STR_DEV_STATUS_REQ);
            break;

        case FSM_TEST_MMI_DEV_CTRL_LED_ONOFF:
            _perform_mmi_test(MMI_TEST_STR_DEV_CTRL_LED_ONOFF);
            break;
            
        default:
            SYS_LOG_ERR("Please Define the test");
            break;
    }
    
    if (FSM_TEST_MODE_SINGLE == fsm_test_mode) {
        _fsm_test_stop();
    }
    else {
        if (++fsm_task_cur_test <= FSM_TEST_MAX) {
            _fsm_proc_test();
        }
        else {
            _fsm_test_stop();
        }
    }
    
    return HAL_OK;
}

static HAL_StatusTypeDef _fsm_test_stop(void) {
    _fsm_send_event(TASK_FSM_EVENT_TEST_DONE);
    
    SYS_LOG_INFO("%d Test Stop");
    return HAL_OK;
}

static void _perform_mmi_test(const char *test_str) {
    uint8_t mmi_hex_arr[50] = {
        0, };
    uint8_t mmi_hex_data_len;
    
    _string_to_hex_array(test_str, mmi_hex_arr, strlen(test_str));
    mmi_hex_data_len = strlen(test_str) / 2;
    Task_MMI_Decoder(mmi_hex_arr, mmi_hex_data_len);
}

static void _string_to_hex_array(const char *str, uint8_t *hex_array, size_t hex_array_len) {
    size_t str_len = strlen(str);
    
    /* Check if the input string length is valid (should be even) */
    if (str_len % 2 != 0) {
        printf("Error: The input string length should be even.\n");
        return;
    }
    
    /* Check if the hex array can accommodate the converted values */
    if (hex_array_len < str_len / 2) {
        printf("Error: hex_array is too small to store the converted values.\n");
        return;
    }
    
    /* Read 2 characters at a time and convert */
    for (size_t i = 0; i < str_len; i += 2) {
        char byte_string[3] = {
            str[i],
            str[i + 1],
            '\0' }; /* 2 characters + NULL terminator */
        hex_array[i / 2] = (uint8_t) strtol(byte_string, NULL, 16);
    }
}
