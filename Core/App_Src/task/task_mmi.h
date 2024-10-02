/**
 ******************************************************************************
 * @file           : task_mmi.h
 * @brief          : Header file for task_mmi.c.
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
#ifndef _TASK_MMI_H_
#define _TASK_MMI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Task_MMI_Decoder(uint8_t *p_ch, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* _TASK_MMI_H_ */
