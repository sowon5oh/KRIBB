/**
 ******************************************************************************
 * @file           : led_task.h
 * @brief          : Header file for led_task.c.
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
#ifndef _LED_TASK_H
#define _LED_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void LED_TASK_Init(void);
void LED_TASK_Start(void);
void LED_TASK_Stop(void);
void LED_TASK_MEAS_M1_Start(void);
void LED_TASK_MEAS_M3_Start(void);
void LED_TASK(void);

HAL_StatusTypeDef LED_TASK_SetSchedule(MeasSchedule_t *p_schdeule,
		uint8_t schedule_len);
HAL_StatusTypeDef LED_TASK_SetAdc(MeasAvrReq_t *p_req_info);
HAL_StatusTypeDef LED_TASK_CFG_Change(MeasAvrReq_t *p_req_info);

void LED_TASK_AdcCollectDone(void);

#ifdef __cplusplus
}
#endif

#endif /* _LED_TASK_H */
