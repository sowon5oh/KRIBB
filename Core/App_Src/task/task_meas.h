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
HAL_StatusTypeDef Task_Meas_Apply_Set(MeasSetCat_t set_cat, MeasSetChVal_t ch, uint8_t *p_set_val);
HAL_StatusTypeDef Task_Meas_Get_Set(MeasSetData_t *p_set_val);
HAL_StatusTypeDef Task_Meas_Request(MeasSetChVal_t ch);
void Task_Meas_RequestCb(void);
HAL_StatusTypeDef Task_Meas_Response(void);
HAL_StatusTypeDef Task_Meas_RequestResult(MeasSetChVal_t *p_ch, MeasResultData_t *p_data);
HAL_StatusTypeDef Task_Meas_Get_Result(MeasResultCat_t result_cat, MeasSetChVal_t ch, uint16_t *p_result_val);
HAL_StatusTypeDef Task_Meas_Get_Status(MeasReqStatus_t *p_status_val);
HAL_StatusTypeDef Task_Meas_Ctrl_Led(MeasSetChVal_t ch, MeasSetLedForcedCtrlVal_t ctrl);
HAL_StatusTypeDef Task_Meas_Ctrl_Monitor(MeasSetChVal_t ch, uint8_t *p_set_val);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_MEAS_H_ */
