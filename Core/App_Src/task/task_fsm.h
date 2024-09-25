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

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Task_Fsm_Init(void);
void Task_Fsm_Start(void);
void Task_Fsm_Stop(void);
void Task_Fsm_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_FSM_H_ */
