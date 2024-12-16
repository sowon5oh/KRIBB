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
HAL_StatusTypeDef Task_Meas_Start(void);
HAL_StatusTypeDef Task_Meas_Stop(void);
void Task_Meas_Req_ContinuosMode(void);
void Task_Meas_Req_SingleMode(uint8_t ch, uint16_t cnt);
HAL_StatusTypeDef Task_Meas_Get_AllChResult(MeasResultData_t *p_data);
HAL_StatusTypeDef Task_Meas_Get_SingleChResult(MeasSetChVal_t * p_ch, uint16_t * p_cnt, int16_t *p_recv, int16_t *p_temp);
HAL_StatusTypeDef Task_Meas_Get_Result(MeasResultCat_t result_cat, MeasSetChVal_t ch, uint16_t *p_result_val);
HAL_StatusTypeDef Task_Meas_Get_Status(MeasReqStatus_t *p_status_val);
HAL_StatusTypeDef Task_Meas_Ctrl_Led(MeasSetChVal_t ch, MeasCtrlLedType_t ctrl);
HAL_StatusTypeDef Task_Meas_Ctrl_OpMode(MeasCtrlOpMode_t op_mode);
HAL_StatusTypeDef Task_Meas_Ctrl_Monitor(MeasSetChVal_t ch, uint8_t *p_set_val);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_MEAS_H_ */
