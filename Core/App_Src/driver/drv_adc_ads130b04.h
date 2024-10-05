/**
 ******************************************************************************
 * @file           : drv_adc_ads130b04.c
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
#ifndef _DRV_ADC_ADS130B04_H_
#define _DRV_ADC_ADS130B04_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum {
    DRV_ADS130B04_CH_0 = 0,
    DRV_ADS130B04_CH_1,
    DRV_ADS130B04_CH_2,
    DRV_ADS130B04_CH_3,
    DRV_ADS130B04_CH_ALL = 4,
    DRV_ADS130B04_CH_NUM = 4,
} Ads130b04_chSel_t;

typedef struct {
    uint8_t ch;
    uint16_t Mux_ch;
    MeasSpsSel_t sps;
    uint16_t ads130b04_num;
    uint16_t wait_time;
    bool measFlag;
} ads130b04Data_t;

#define ADS130B04_AVR_BUFF_MAX_NUM	500

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
typedef void (*MeasAvrResultCb_t)(float result);
HAL_StatusTypeDef DRV_ADS130B04_Init(SPI_HandleTypeDef *p_handle);
HAL_StatusTypeDef DRV_ADS130B04_ReqAvr(ads130b04Data_t *p_req_info, MeasAvrResultCb_t cb_fn);
HAL_StatusTypeDef DRV_ADS130B04_Start(void);
HAL_StatusTypeDef DRV_ADS130B04_Stop(void);
void ADC_CFG_Change(ads130b04Data_t *p_req_info);
void ADC_MeasStop(void);
void ADC_GetSetting(uint8_t *ch, uint8_t *sps, uint16_t *samples, uint16_t *wait_time);

#ifdef __cplusplus
}
#endif

#endif /* _DRV_ADC_ADS130B04_H_ */
