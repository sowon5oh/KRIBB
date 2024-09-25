/**
 ******************************************************************************
 * @file           : data_type.h
 * @brief          : Header file containing shared data types, structures,
 *                   and common constants for the project.
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
#ifndef _DATA_TYPE_H_
#define _DATA_TYPE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

#define PROTOCOL_DATA_MAX_SIZE			20
#define LED_SCHEDULE_LEN_MAX 			4
#define LED_MEAS_M1_RESULT_DATA_SIZE   	5	/* led ch: 1byte, adc average: 4byte */
#define LED_MEAS_M3_RESULT_DATA_SIZE   	4	/* adc average: 4byte */

typedef struct {
    uint8_t led_ch;
    float avr_adc;
} MeasM1Result_t;

typedef struct {
    float avr_adc;
} MeasM3Result_t;

typedef union {
    uint8_t bVal[4];
    float fVal;
} FtoB;

typedef struct {
    uint8_t led_ch;
    uint16_t led_data;
} MeasSchedule_t;

typedef enum {
    ADC_CH_0 = 0,
    ADC_CH_1,
    ADC_CH_2,
    ADC_CH_3, //MUX
    ADC_CH_NUM,
} MeasChSel_t;

typedef enum {
    ADC_SPS_32K = 0,
    ADC_SPS_16K,
    ADC_SPS_8K,
    ADC_SPS_4K,
    ADC_SPS_2K,
    ADC_SPS_1K, /* Default */
    ADC_SPS_0_5K,
    ADC_SPS_0_25K,
} MeasSpsSel_t;

typedef struct {
    MeasChSel_t ch;
    uint16_t Mux_ch;
    MeasSpsSel_t sps;
    uint16_t adc_num;
    uint16_t wait_time;
    bool measFlag;
} MeasAvrReq_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _DATA_TYPE_H_ */
