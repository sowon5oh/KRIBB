/**
 ******************************************************************************
 * @file           : task_fsm.h
 * @brief          : Header file for Task_Fsm.c.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _TASK_FSM_H_
#define _TASK_FSM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum {
    TASK_FSM_STATE_IDLE = 0,
    TASK_FSM_STATE_READY,
    TASK_FSM_STATE_TEST,
    TASK_FSM_STATE_NUM,
} FsmState_t;

typedef enum {
    TASK_FSM_EVENT_INIT_DONE = 0,
    TASK_FSM_EVENT_TEST_REQ,
    TASK_FSM_EVENT_TEST_DONE,
    TASK_FSM_EVENT_TIMEOUT,
    TASK_FSM_EVENT_MAX = TASK_FSM_EVENT_TIMEOUT,
} FsmEvent_t;

typedef enum {
    /* Device Test */
    FSM_TEST_DEVICE_LED_ONOFF = 0,
    FSM_TEST_DEVICE_HEATER_ONOFF,
    FSM_TEST_DEVICE_READ_TEMP,

    /* Measure Request */

    /* MMI Test */
    FSM_TEST_MMI_WHO_AM_I_DEV_INFO,
    FSM_TEST_MMI_WHO_AM_I,
    FSM_TEST_MMI_DEV_INFO,
    FSM_TEST_MMI_SET_REQ_ALL_SETTINGS,
    FSM_TEST_MMI_SET_TEMP_ONOFF,
    FSM_TEST_MMI_SET_LED_ON_TIME,
    FSM_TEST_MMI_SET_LED_ON_LEVEL,
    FSM_TEST_MMI_SET_ADC_SAMPLE_CNT,
    FSM_TEST_MMI_SET_ADC_DELAY_MS,
    FSM_TEST_MMI_SET_STABLE_TEMPERATURE,
    FSM_TEST_MMI_REQ_MEASURE,
    FSM_TEST_MMI_REQ_TEMPERATURE,
    FSM_TEST_MMI_REQ_RECV_PD,
    FSM_TEST_MMI_REQ_MONITOR_PD,
    FSM_TEST_MMI_DEV_CTRL_LED_ONOFF,
    FSM_TEST_MMI_DEV_STATUS_REQ,
    /* Etc */
    FSM_TEST_MAX = FSM_TEST_MMI_DEV_STATUS_REQ,
/* Add.. */
} FsmTaskTestType_t;

typedef enum {
    FSM_TEST_MODE_SINGLE = 0,
    FSM_TEST_MODE_SEQUENCE,
} FsmTaskTestMode_t;

/* Test MMI String */
#define MMI_TEST_STR_WHO_AM_I_DEV_INFO      "C0000155000000D8C2"
#define MMI_TEST_STR_WHO_AM_I               "C0000155AA5A00DCC2"
#define MMI_TEST_STR_DEV_INFO               "C0000155010100DAC2"

/* Settings */
#define MMI_TEST_STR_SET_REQ_ALL_SETTINGS   "C000010A0000008DC2"

#define MMI_TEST_STR_SET_TEMP_ONOFF         "C000010A01040193C2"
#define MMI_TEST_STR_SET_LED_ON_TIME        "C000020A020103E87CC2"

#define MMI_TEST_STR_SET_LED_ON_LEVEL       "C000020A0301000193C2" /* CH 1: 0x0001 (1    dgit) */
//#define MMI_TEST_STR_SET_LED_ON_LEVEL       "C000020A03010FFFA0C2" /* CH 1: 0x0FFF (4095 dgit) */

//#define MMI_TEST_STR_SET_ADC_SAMPLE_CNT     "C000020A0401000194C2" /* CH 1  : 0x0001 (1    ms) */
#define MMI_TEST_STR_SET_ADC_SAMPLE_CNT     "C000020A040103E87EC2" /* CH 1  : 0x03E8 (1000 ms) */
//#define MMI_TEST_STR_SET_ADC_SAMPLE_CNT     "C000020A0404000197C2" /* CH ALL: 0x0001 (1    ms) */

//#define MMI_TEST_STR_SET_ADC_DELAY_MS       "C000020A0501000195C2" /* CH 1: 0x0001 */
//#define MMI_TEST_STR_SET_ADC_DELAY_MS       "C000020A050103E87FC2" /* CH 1  : 0x03E8 (1000 ms) */
#define MMI_TEST_STR_SET_ADC_DELAY_MS       "C000020A0504000198C2" /* CH 1: 0x0001 */

#define MMI_TEST_STR_SET_STABLE_TEMPERATURE "C000020A06010BB858C2"

/* Measure Request */
#define MMI_TEST_STR_REQ_ALL_DATA           "C000010B0000008EC2"

#define MMI_TEST_STR_REQ_TEMPERATURE        "C000010B01010090C2" /* CH 1 */
//#define MMI_TEST_STR_REQ_TEMPERATURE        "C000010B01040093C2" /* CH ALL */

#define MMI_TEST_STR_REQ_RECV_PD            "C000010B02010091C2" /* CH 1 */
//#define MMI_TEST_STR_REQ_RECV_PD            "C000010B02040094C2" /* CH ALL */

#define MMI_TEST_STR_REQ_MONITOR_PD         "C000010B03010092C2" /* CH 1 */
//#define MMI_TEST_STR_REQ_MONITOR_PD         "C000010B03040095C2" /* CH ALL */

#define MMI_TEST_STR_DEV_STATUS_REQ         "C000010C00008FC2"

//#define MMI_TEST_STR_DEV_CTRL_LED_ONOFF     "C000010D01040095C2" /* CH ALL: Forced OFF */
#define MMI_TEST_STR_DEV_CTRL_LED_ONOFF     "C000010D01040196C2" /* CH ALL: Forced ON */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Task_Fsm_Init(void);
void Task_Fsm_Process(void);
void Task_Fsm_StartTest(FsmTaskTestType_t test, FsmTaskTestMode_t mode);
void Task_Fsm_SendEvent(FsmEvent_t event);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_FSM_H_ */
