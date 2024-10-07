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
/*
 * MMI Protocol Index
 *
 *  [STX] [LEN (2 Bytes)] [CMD1] [CMD2] [CMD3] [Data ... N-Byte] [ChkSum] [ETX]
 *
 *  STX   : Start of Text
 *  LEN   : Length of the Data Bytes (2 Bytes)
 *  CMD1  : Command byte 1
 *  CMD2  : Command byte 2
 *  CMD3  : Command byte 3
 *  Data  : N number of data bytes
 *  ChkSum: Checksum for error detection
 *  ETX   : End of Text
 *
 */
#define MMI_PROTOCOL_STX            0xC0
#define MMI_PROTOCOL_SPC            0xDB
#define MMI_PROTOCOL_ETX            0xC2
#define MMI_PROTOCOL_CHKSUM_INIT    0xC2
#define MMI_PROTOCOL_WHO_AM_I_VAL   0x55AA5AA5
#define MMI_PROTOCOL_WHO_AM_I_LEN   (4)

#define MMI_PROTOCOL_IDX_STX        0 /* 1 Byte */
#define MMI_PROTOCOL_IDX_LEN        1 /* 2 Byte */
#define MMI_PROTOCOL_IDX_CMD1       3 /* 1 Byte */
#define MMI_PROTOCOL_IDX_CMD2       4 /* 1 Byte */
#define MMI_PROTOCOL_IDX_CMD3       5 /* 1 Byte */
#define MMI_PROTOCOL_IDX_DATA       6 /* Max 12 Byte */

#define MMI_PROTOCOL_HEADER_LEN_MAX 7
#define MMI_PROTOCOL_TX_DATA_LEN_MAX   30
#define MMI_PROTOCOL_RX_DATA_LEN_MAX   2
#define MMI_PROTOCOL_FOOTER_LEN_MAX 2
#define MMI_PROTOCOL_TX_MSG_LEN_MAX MMI_PROTOCOL_HEADER_LEN_MAX + MMI_PROTOCOL_TX_DATA_LEN_MAX + MMI_PROTOCOL_FOOTER_LEN_MAX
#define MMI_PROTOCOL_RX_MSG_LEN_MAX MMI_PROTOCOL_HEADER_LEN_MAX + MMI_PROTOCOL_RX_DATA_LEN_MAX + MMI_PROTOCOL_FOOTER_LEN_MAX

#define MMI_PROTOCOL_BYTE_STUFF_PADDING     0xDB
#define MMI_PROTOCOL_BYTE_STUFF_C0_REPLACE  0xDC
#define MMI_PROTOCOL_BYTE_STUFF_DB_REPLACE  0xDD
#define MMI_PROTOCOL_BYTE_STUFF_C2_REPLACE  0xDE

#define MMI_PROTOCOL_CMD1_RX       0x00
#define MMI_PROTOCOL_CMD1_TX       0x10

/* RX Command Group 1: Device Info */
#define MMI_CMD1_INFO                                0x55
#define MMI_CMD1_INFO_DATA_MAX_LEN                   MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE_DATA_LEN
#define MMI_CMD2_INFO_WHO_AM_I_AND_DEVICE            0x00
#define MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE            0x00
#define MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE_DATA_LEN   (8)

#define MMI_CMD2_INFO_WHO_AM_I           0xAA
#define MMI_CMD3_INFO_WHO_AM_I           0x5A
#define MMI_CMD3_INFO_WHO_AM_I_DATA_LEN (4)

#define MMI_CMD2_INFO_DEVICE        0x01
#define MMI_CMD3_INFO_DEVICE        0x01
#define MMI_CMD3_INFO_DEVICE_DATA_LEN (4)

#define MMI_CMD2_RESERVED_0         0x02
#define MMI_CMD2_RESERVED_1         0x03

/* RX Command Group 2: Set for Measuring */
#define MMI_CMD1_MEAS_SET           0x0A
#define MMI_CMD1_MEAS_SET_MAX_DATA_LEN (2)

#define MMI_CMD2_MEAS_SET_VAL_REQ       0x00
#define MMI_CMD3_MEAS_SET_VAL_REQ       0x00

#define MMI_CMD2_SET_MEAS_TEMP            0x01
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_CH1  0x01
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_CH2  0x02
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_CH3  0x03
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_ALL  0x04
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_DATA_LEN (1)

#define MMI_CMD2_SET_MEAS_LED_ON_TIME          0x02
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_CH1      0x01
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_CH2      0x02
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_CH3      0x03
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_ALL      0x04
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_DATA_LEN (2)

#define MMI_CMD2_SET_MEAS_LED_LEVEL       0x03
#define MMI_CMD3_SET_MEAS_LED_LEVEL_CH1   0x01
#define MMI_CMD3_SET_MEAS_LED_LEVEL_CH2   0x02
#define MMI_CMD3_SET_MEAS_LED_LEVEL_CH3   0x03
#define MMI_CMD3_SET_MEAS_LED_LEVEL_ALL   0x04
#define MMI_CMD3_SET_MEAS_LED_LEVEL_DATA_LEN (2)

#define MMI_CMD2_SET_MEAS_ADC_SAMPLE_CNT      0x04
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_CH1  0x01
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_CH2  0x02
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_CH3  0x03
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_ALL  0x04
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_DATA_LEN (2)

#define MMI_CMD2_SET_MEAS_ADC_DELAY_MS        0x05
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_CH1    0x01
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_CH2    0x02
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_CH3    0x03
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_ALL    0x04
#define MMI_CMD3_SET_MEAS_ADC_DELAY_DATA_LEN (2)

/* RX Command Group 3: Request Measuring */
#define MMI_CMD1_REQ_MEAS                0x0B
#define MMI_CMD1_REQ_MEAS_MAX_DATA_LEN   (18)

#define MMI_CMD2_REQ_MEAS_ALL            0x00
#define MMI_CMD3_REQ_MEAS_ALL            0x00
#define MMI_CMD3_REQ_MEAS_ALL_DATA_LEN   (18)

#define MMI_CMD2_REQ_MEAS_TEMP_ADC       		0x01
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_CH1   		0x01
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_CH2   		0x02
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_CH3   		0x03
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_ALL          0x04
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_MAX_DATA_LEN (6)

#define MMI_CMD2_REQ_MEAS_PD_ADC              0x02
#define MMI_CMD3_REQ_MEAS_PD_ADC_CH1          0x01
#define MMI_CMD3_REQ_MEAS_PD_ADC_CH2          0x02
#define MMI_CMD3_REQ_MEAS_PD_ADC_CH3          0x03
#define MMI_CMD3_REQ_MEAS_PD_ADC_ALL          0x04
#define MMI_CMD3_REQ_MEAS_PD_ADC_MAX_DATA_LEN (6)

#define MMI_CMD2_REQ_MEAS_MONITOR_ADC     0x03
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_CH1 0x01
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_CH2 0x02
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_CH3 0x03
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_ALL 0x04
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_MAX_DATA_LEN (6)

#define MMI_CMD2_REQ_MEAS_START          0x04
#define MMI_CMD3_REQ_MEAS_START_CH1      0x01
#define MMI_CMD3_REQ_MEAS_START_CH2      0x02
#define MMI_CMD3_REQ_MEAS_START_CH3      0x03

/* RX Command Group 4: Request Device Status */
#define MMI_CMD1_REQ_DEVICE_STATUS       0x0C
#define MMI_CMD2_REQ_DEVICE_STATUS       0x00
#define MMI_CMD3_REQ_DEVICE_STATUS       0x00

/* RX Command Group 5: Device Control */
#define MMI_CMD1_CTRL_DEVICE               0x0D
#define MMI_CMD1_CTRL_DEVICE_MAX_DATA_LEN (1)

#define MMI_CMD2_CTRL_DEVICE_LED          0x01
#define MMI_CMD3_CTRL_DEVICE_LED_CH1      0x01
#define MMI_CMD3_CTRL_DEVICE_LED_CH2      0x02
#define MMI_CMD3_CTRL_DEVICE_LED_CH3      0x03
#define MMI_CMD3_CTRL_DEVICE_LED_ALL      0x04
#define MMI_CMD3_CTRL_DEVICE_LED_DATA_LEN (1)

#define MMI_CMD2_CTRL_DEVICE_MONITOR      0x02
#define MMI_CMD3_CTRL_DEVICE_MONITOR_CH1  0x01
#define MMI_CMD3_CTRL_DEVICE_MONITOR_CH2  0x02
#define MMI_CMD3_CTRL_DEVICE_MONITOR_CH3  0x03
#define MMI_CMD3_CTRL_DEVICE_MONITOR_ALL  0x04
#define MMI_CMD3_CTRL_DEVICE_MONITOR_LEN  (1)

#define PROTOCOL_DATA_MAX_SIZE          20

typedef union {
    uint8_t bVal[4];
    float fVal;
} FtoB;


typedef enum {
    MEAS_SET_CAT_TEMP_ON_OFF = 0,
    MEAS_SET_CAT_LED_ON_TIME,
    MEAS_SET_CAT_LED_ON_LEVEL,
    MEAS_SET_CAT_ADC_SAMPLE_CNT,
    MEAS_SET_CAT_ADC_ON_DELAY,
    MEAS_SET_CAT_MAX = MEAS_SET_CAT_ADC_ON_DELAY,
} MeasSetCat_t;

typedef enum {
    MEAS_RESULT_CAT_TEMP_ADC = 0,
    MEAS_RESULT_CAT_PD_ADC,
    MEAS_RESULT_CAT_MONITOR_ADC,
    MEAS_RESULT_CAT_MAX = MEAS_RESULT_CAT_MONITOR_ADC,
} MeasResultCat_t;

typedef enum {
    MEAS_SET_CH_1 = 0x01,
    MEAS_SET_CH_2 = 0x02,
    MEAS_SET_CH_3 = 0x03,
    MEAS_SET_CH_ALL = 0x04,
    MEAS_SET_CH_MAX = MEAS_SET_CH_ALL,
} MeasSetChVal_t;

typedef enum {
    TEMP_CTRL_OFF = 0x00,
    TEMP_CTRL_AUTO_ON = 0x01,
    TEMP_CTRL_FORCE_ON = 0x02, /* for test */
} MeasSetTempCtrlVal_t;

#define CH_NUM  3
#define CH1_IDX 0
#define CH2_IDX 1
#define CH3_IDX 2

#define MEAS_SET_DEFAULT_TEMP_CTRL_ON    TEMP_CTRL_OFF

#define MEAS_SET_DEFAULT_LED_ON_TIME_MS  0
#define MEAS_SET_MIN_LED_ON_TIME_MS      0
#define MEAS_SET_MAX_LED_ON_TIME_MS      1000

#define MEAS_SET_DEFAULT_LED_ON_LEVEL    0
#define MEAS_SET_MIN_LED_ON_LEVEL        0
#define MEAS_SET_MAX_LED_ON_LEVEL        65535

#define MEAS_SET_DEFAULT_ADC_SAMPLE_CNT  0
#define MEAS_SET_MIN_ADC_SAMPLE_CNT      0
#define MEAS_SET_MAX_ADC_SAMPLE_CNT      1000

#define MEAS_SET_DEFAULT_ADC_DELAY_MS    0
#define MEAS_SET_MIN_ADC_DELAY_MS        0
#define MEAS_SET_MAX_ADC_DELAY_MS        1000

typedef struct {
    MeasSetTempCtrlVal_t temp_ctrl_on[CH_NUM]; /* Ch1 - CH3 */
    uint16_t led_on_time[CH_NUM];
    uint16_t led_on_level[CH_NUM];
    uint16_t adc_sample_cnt[CH_NUM];
    uint16_t adc_delay_ms[CH_NUM];
} MeasSetData_t;

typedef struct {
    uint16_t temperature_data[CH_NUM];
    uint16_t recv_pd_data[CH_NUM];
    uint16_t recv_pd_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];
    uint16_t monitor_data[CH_NUM];
    uint16_t monitor_buff[CH_NUM][MEAS_SET_MAX_ADC_SAMPLE_CNT];
} MeasResultData_t;

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

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _DATA_TYPE_H_ */
