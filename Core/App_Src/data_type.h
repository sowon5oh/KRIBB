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

#define MMI_PROTOCOL_IDX_STX        0
#define MMI_PROTOCOL_IDX_LEN        1
#define MMI_PROTOCOL_IDX_CMD1       3
#define MMI_PROTOCOL_IDX_CMD2       4
#define MMI_PROTOCOL_IDX_CMD3       5
#define MMI_PROTOCOL_IDX_DATA       6

#define MMI_PROTOCOL_BYTE_STUFF_PADDING     0xDB
#define MMI_PROTOCOL_BYTE_STUFF_C0_REPLACE  0xDC
#define MMI_PROTOCOL_BYTE_STUFF_DB_REPLACE  0xDD
#define MMI_PROTOCOL_BYTE_STUFF_C2_REPLACE  0xDE

#define MMI_PROTOCOL_CMD1_RX       0x00
#define MMI_PROTOCOL_CMD1_TX       0x10

/* RX Command Group 1: Device Info */
#define MMI_CMD1_INFO        0x55
#define MMI_CMD1_INFO_DATA_MAX_LEN        MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE_DATA_LEN
#define MMI_CMD2_INFO_WHO_AM_I_AND_DEVICE   0x00
#define MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE   0x00
#define MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE_DATA_LEN  (8)
#define MMI_CMD2_INFO_WHO_AM_I           0xAA
#define MMI_CMD3_INFO_WHO_AM_I           0x5A
#define MMI_CMD3_INFO_WHO_AM_I_DATA_LEN (4)
#define MMI_CMD2_INFO_DEVICE        0x01
#define MMI_CMD3_INFO_DEVICE        0x01
#define MMI_CMD3_INFO_DEVICE_DATA_LEN (4)
#define MMI_CMD2_RESERVED_0         0x02
#define MMI_CMD2_RESERVED_1         0x03

/* RX Command Group 2: Set for Measuring */
#define MMI_CMD1_MEAS_SET            0x0A
#define MMI_CMD2_MEAS_SET_REQ       0x00
#define MMI_CMD3_MEAS_SET_REQ       0x00
#define MMI_CMD2_SET_MEAS_TEMP            0x01
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_CH1  0x01
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_CH2  0x02
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_CH3  0x03
#define MMI_CMD3_SET_MEAS_TEMP_ONOFF_ALL  0x04
#define MMI_CMD2_SET_MEAS_LED_ON_TIME     0x02
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_CH1 0x01
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_CH2 0x02
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_CH3 0x03
#define MMI_CMD3_SET_MEAS_LED_ON_TIME_ALL 0x04
#define MMI_CMD2_SET_MEAS_LED_LEVEL       0x03
#define MMI_CMD3_SET_MEAS_LED_LEVEL_CH1   0x01
#define MMI_CMD3_SET_MEAS_LED_LEVEL_CH2   0x02
#define MMI_CMD3_SET_MEAS_LED_LEVEL_CH3   0x03
#define MMI_CMD3_SET_MEAS_LED_LEVEL_ALL   0x04
#define MMI_CMD2_SET_MEAS_ADC_SAMPLE_CNT      0x04
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_CH1  0x01
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_CH2  0x02
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_CH3  0x03
#define MMI_CMD3_SET_MEAS_ADC_SAMPLE_CNT_ALL  0x04
#define MMI_CMD2_SET_MEAS_ADC_DELAY_MS        0x05
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_CH1    0x01
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_CH2    0x02
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_CH3    0x03
#define MMI_CMD3_SET_MEAS_ADC_DELAY_MS_ALL    0x04

/* RX Command Group 3: Request Measuring */
#define MMI_CMD1_REQ_MEAS                0x0B
#define MMI_CMD2_REQ_MEAS_TEMP_ADC       0x01
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_CH1   0x01
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_CH2   0x02
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_CH3   0x03
#define MMI_CMD3_REQ_MEAS_TEMP_ADC_ALL   0x04
#define MMI_CMD2_REQ_MEAS_PD_ADC         0x02
#define MMI_CMD3_REQ_MEAS_PD_ADC_CH1     0x01
#define MMI_CMD3_REQ_MEAS_PD_ADC_CH2     0x02
#define MMI_CMD3_REQ_MEAS_PD_ADC_CH3     0x03
#define MMI_CMD3_REQ_MEAS_PD_ADC_ALL     0x04
#define MMI_CMD2_REQ_MEAS_MONITOR_ADC    0x03
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_CH1 0x01
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_CH2 0x02
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_CH3 0x03
#define MMI_CMD3_REQ_MEAS_MONITOR_ADC_ALL 0x04
#define MMI_CMD2_REQ_MEAS_START          0x04
#define MMI_CMD3_REQ_MEAS_START_CH1      0x01
#define MMI_CMD3_REQ_MEAS_START_CH2      0x02
#define MMI_CMD3_REQ_MEAS_START_CH3      0x03

/* RX Command Group 4: Request Device Status */
#define MMI_CMD1_REQ_DEVICE_STATUS       0x0C
#define MMI_CMD2_REQ_DEVICE_STATUS       0x00
#define MMI_CMD3_REQ_DEVICE_STATUS       0x00

/* RX Command Group 5: Device Control */
#define MMI_CMD1_CTRL_DEVICE             0x0D
#define MMI_CMD2_CTRL_DEVICE_LED         0x01
#define MMI_CMD3_CTRL_DEVICE_LED_CH1     0x01
#define MMI_CMD3_CTRL_DEVICE_LED_CH2     0x02
#define MMI_CMD3_CTRL_DEVICE_LED_CH3     0x03
#define MMI_CMD3_CTRL_DEVICE_LED_ALL     0x04
#define MMI_CMD2_CTRL_DEVICE_MONITOR     0x02
#define MMI_CMD3_CTRL_DEVICE_MONITOR_CH1 0x01
#define MMI_CMD3_CTRL_DEVICE_MONITOR_CH2 0x02
#define MMI_CMD3_CTRL_DEVICE_MONITOR_CH3 0x03
#define MMI_CMD3_CTRL_DEVICE_MONITOR_ALL 0x04

#define PROTOCOL_DATA_MAX_SIZE          20

typedef union {
    uint8_t bVal[4];
    float fVal;
} FtoB;

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
} MeasReqData_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _DATA_TYPE_H_ */
