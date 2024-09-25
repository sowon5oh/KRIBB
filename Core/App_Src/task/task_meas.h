/**
 ******************************************************************************
 * @file           : task_meas.h
 * @brief          : Header file for Task_Meas.c.
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
#ifndef _TASK_MEAS_H_
#define _TASK_MEAS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Task_Meas_Init(void);
void Task_Meas_Start(void);
void Task_Meas_Stop(void);
void Task_Meas_MEAS_M1_Start(void);
void Task_Meas_MEAS_M3_Start(void);
void Task_Meas_Process(void);

HAL_StatusTypeDef Task_Meas_SetSchedule(MeasSchedule_t *p_schdeule,
		uint8_t schedule_len);
HAL_StatusTypeDef Task_Meas_SetAdc(MeasAvrReq_t *p_req_info);
HAL_StatusTypeDef Task_Meas_CFG_Change(MeasAvrReq_t *p_req_info);

void Task_Meas_AdcCollectDone(void);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_MEAS_H_ */
