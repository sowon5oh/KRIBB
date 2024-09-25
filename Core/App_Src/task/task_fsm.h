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
    TASK_FSM_STATE_MEAS,
    TASK_FSM_STATE_MAX
} FsmState_t;

typedef enum {
    TASK_FSM_EVENT_MEAS_REQ = 0,
    TASK_FSM_EVENT_MEAS_DONE,
    TASK_FSM_EVENT_TIMEOUT,
    TASK_FSM_EVENT_MAX
} FsmEvent_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Task_Fsm_Init(void);
void Task_Fsm_Start(void);
void Task_Fsm_Stop(void);
void Task_Fsm_Process(void);
void Task_Fsm_SendEvent(FsmEvent_t event);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_FSM_H_ */
