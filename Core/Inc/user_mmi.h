/**
 ******************************************************************************
 * @file           : user_mmi.h
 * @brief          : Header file for user_mmi.c.
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
#ifndef _USER_MMI_H_
#define _USER_MMI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void MMI_Decoder(uint8_t *p_ch, uint16_t len);
void MMI_M1ResultSender(MeasM1Result_t *p_result, uint8_t result_num);
void MMI_M3ResultSender(MeasM3Result_t *p_result);
void MMI_Sender(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t *p_data,
		uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* _USER_MMI_H_ */
