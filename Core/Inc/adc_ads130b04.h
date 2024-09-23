/**
 ******************************************************************************
 * @file           : adc_ads130b04.c
 * @brief          : Header file for adc_ads130b04.c.
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
#ifndef _ADC_ADS130B04_H
#define _ADC_ADS130B04_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define ADC_AVR_BUFF_MAX_NUM	500

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
typedef void (*MeasAvrResultCb_t)(float result);
HAL_StatusTypeDef ADC_Init(SPI_HandleTypeDef *p_handle);
HAL_StatusTypeDef ADC_ReqAvr(MeasAvrReq_t *p_req_info, MeasAvrResultCb_t cb_fn);
HAL_StatusTypeDef ADC_Start(void);
HAL_StatusTypeDef ADC_Stop(void);
void ADC_CFG_Change(MeasAvrReq_t *p_req_info);
void ADC_MeasStop(void);
void ADC_GetSetting(uint8_t *ch, uint8_t *sps, uint16_t *samples,
		uint16_t *wait_time);

#ifdef __cplusplus
}
#endif

#endif /* _ADC_ADS130B04_H */
