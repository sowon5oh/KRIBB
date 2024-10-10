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
    DRV_ADS130B04_CH_3_0,
    DRV_ADS130B04_CH_3_1,
    DRV_ADS130B04_CH_3_2,
    DRV_ADS130B04_CH_NUM = 6,
    DRV_ADS130B04_CH_3 = DRV_ADS130B04_CH_3_0,
    DRV_ADS130B04_CH_MAX = DRV_ADS130B04_CH_NUM,
} Ads130b04ChSel_t;

typedef enum {
    DRV_ADS130B04_MUX_CH_0 = 0,
    DRV_ADS130B04_MUX_CH_1,
    DRV_ADS130B04_MUX_CH_2,
    DRV_ADS130B04_MUX_CH_NUM = 3,
    DRV_ADS130B04_MUX_CH_MAX = DRV_ADS130B04_MUX_CH_NUM,
} Ads130b04Ch3MuxCh_t;

typedef struct {
    uint8_t ch;
    uint16_t Mux_ch;
    MeasSpsSel_t sps;
    uint16_t ads130b04_num;
    uint16_t wait_time;
    bool measFlag;
} ads130b04Data_t;

#define ADS130B04_AVR_BUFF_MAX_NUM	500

typedef void (*MeasAvrResultCb_t)(float result);
/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef DRV_ADS130B04_Init(SPI_HandleTypeDef *p_handle);
HAL_StatusTypeDef DRV_ADS130B04_Start(void);
HAL_StatusTypeDef DRV_ADS130B04_Stop(void);
HAL_StatusTypeDef DRV_ADS130B04_GetData(Ads130b04ChSel_t ch, int16_t *p_data);

#ifdef __cplusplus
}
#endif

#endif /* _DRV_ADC_ADS130B04_H_ */
