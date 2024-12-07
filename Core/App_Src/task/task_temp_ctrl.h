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
#ifndef _TASK_TEMP_CTRL_H_
#define _TASK_TEMP_CTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Task_TempCtrl_Init(void);
HAL_StatusTypeDef Task_TempCtrl_Start(void);
HAL_StatusTypeDef Task_TempCtrl_Stop(void);
HAL_StatusTypeDef Task_TempCtrl_SetStableTemp(float stable_temp);
HAL_StatusTypeDef Task_TempCtrl_GetCurTemp(MeasCh_t ch, float *p_temp);
HAL_StatusTypeDef Task_TempCtrl_SetCtrlType(MeasCh_t ch, MeasSetTempCtrlTypeVal_t type);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_TEMP_CTRL_H_ */
