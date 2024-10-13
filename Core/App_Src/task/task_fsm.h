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
    FSM_TEST_MMI_DEVICE_CTRL_LED = 0,
/* Add.. */
} FsmTaskTestType_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Task_Fsm_Init(void);
void Task_Fsm_Process(void);
void Task_Fsm_SendEvent(FsmEvent_t event);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_FSM_H_ */
