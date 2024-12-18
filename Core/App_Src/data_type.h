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
#define MMI_PROTOCOL_STX            		0xC0
#define MMI_PROTOCOL_SPC            		0xDB
#define MMI_PROTOCOL_ETX            		0xC2
#define MMI_PROTOCOL_CHKSUM_INIT    		0xC2
#define MMI_PROTOCOL_WHO_AM_I_VAL   		0x55AA5AA5
#define MMI_PROTOCOL_WHO_AM_I_LEN   		(4)

#define MMI_PROTOCOL_IDX_STX        		0 /* 1 Byte */
#define MMI_PROTOCOL_IDX_LEN        		1 /* 2 Byte */
#define MMI_PROTOCOL_IDX_CMD1       		3 /* 1 Byte */
#define MMI_PROTOCOL_IDX_CMD2       		4 /* 1 Byte */
#define MMI_PROTOCOL_IDX_CMD3       		5 /* 1 Byte */
#define MMI_PROTOCOL_IDX_DATA       		6 /* Max 12 Byte */

#define MMI_PROTOCOL_HEADER_LEN_MAX  		7
#define MMI_PROTOCOL_TX_DATA_LEN_MAX 		30
#define MMI_PROTOCOL_RX_DATA_LEN_MAX 		20
#define MMI_PROTOCOL_FOOTER_LEN_MAX  		2
#define MMI_PROTOCOL_TX_MSG_LEN_MAX  		MMI_PROTOCOL_HEADER_LEN_MAX + MMI_PROTOCOL_TX_DATA_LEN_MAX + MMI_PROTOCOL_FOOTER_LEN_MAX
#define MMI_PROTOCOL_RX_MSG_LEN_MAX  		MMI_PROTOCOL_HEADER_LEN_MAX + MMI_PROTOCOL_RX_DATA_LEN_MAX + MMI_PROTOCOL_FOOTER_LEN_MAX

#define MMI_PROTOCOL_BYTE_STUFF_PADDING     0xDB
#define MMI_PROTOCOL_BYTE_STUFF_C0_REPLACE  0xDC
#define MMI_PROTOCOL_BYTE_STUFF_DB_REPLACE  0xDD
#define MMI_PROTOCOL_BYTE_STUFF_C2_REPLACE  0xDE

#define MMI_PROTOCOL_CMD1_RX                0x00
#define MMI_PROTOCOL_CMD1_TX              	0x10

/* RX Command Group 1: Device Info */
#define MMI_CMD1_INFO                              0x55
#define MMI_CMD1_INFO_DATA_MAX_LEN                 MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE_DATA_LEN

#define MMI_CMD2_INFO_WHO_AM_I_AND_DEVICE_I_W_RESP 0x00
#define MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE          0x00
#define MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE_DATA_LEN (8)

#define MMI_CMD2_INFO_WHO_AM_I_W_RESP    0xAA
#define MMI_CMD3_INFO_WHO_AM_I           0x5A
#define MMI_CMD3_INFO_WHO_AM_I_DATA_LEN (4)

#define MMI_CMD2_INFO_DEVICE_I_W_RESP    0x01
#define MMI_CMD3_INFO_DEVICE             0x01
#define MMI_CMD3_INFO_DEVICE_DATA_LEN    (4)

/* RX Command Group 2: Set for Measuring */
#define MMI_CMD1_MEAS_SET                   0x0A
#define MMI_CMD1_MEAS_SET_RESP              0x1A
#define MMI_CMD1_MEAS_SET_MAX_DATA_LEN      MMI_CMD3_MEAS_SET_VAL_REQ_DATA_LEN

#define MMI_CMD2_MEAS_SET_VAL_REQ_W_RESP    0x00
#define MMI_CMD3_MEAS_SET_VAL_REQ           0x00
#define MMI_CMD3_MEAS_SET_VAL_REQ_DATA_LEN  (29)
//#define MMI_CMD3_MEAS_SET_VAL_REQ_DATA_LEN  (31) /* Add Temperature offset */

#define MMI_CMD2_MEAS_SET_TEMP           	  0x01
#define MMI_CMD3_MEAS_SET_TEMP_ONOFF_CH1 	  0x01
#define MMI_CMD3_MEAS_SET_TEMP_ONOFF_CH2 	  0x02
#define MMI_CMD3_MEAS_SET_TEMP_ONOFF_CH3  	  0x03
#define MMI_CMD3_MEAS_SET_TEMP_ONOFF_ALL  	  0x04
#define MMI_CMD3_MEAS_SET_TEMP_ONOFF_DATA_LEN (1)

#define MMI_CMD2_MEAS_SET_LED_ON_TIME          0x02
#define MMI_CMD3_MEAS_SET_LED_ON_TIME_CH1      0x01
#define MMI_CMD3_MEAS_SET_LED_ON_TIME_CH2      0x02
#define MMI_CMD3_MEAS_SET_LED_ON_TIME_CH3      0x03
#define MMI_CMD3_MEAS_SET_LED_ON_TIME_ALL      0x04
#define MMI_CMD3_MEAS_SET_LED_ON_TIME_DATA_LEN (2)

#define MMI_CMD2_MEAS_SET_LED_LEVEL_W_RESP    0x03
#define MMI_CMD3_MEAS_SET_LED_LEVEL_CH1   	  0x01
#define MMI_CMD3_MEAS_SET_LED_LEVEL_CH2   	  0x02
#define MMI_CMD3_MEAS_SET_LED_LEVEL_CH3   	  0x03
#define MMI_CMD3_MEAS_SET_LED_LEVEL_ALL   	  0x04
#define MMI_CMD3_MEAS_SET_LED_LEVEL_DATA_LEN (2)
#define MMI_CMD2_MEAS_SET_RESP_LED_LEVEL      0x13

#define MMI_CMD2_MEAS_SET_ADC_SAMPLE_CNT          0x04
#define MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_CH1      0x01
#define MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_CH2      0x02
#define MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_CH3      0x03
#define MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_ALL      0x04
#define MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_DATA_LEN (2)

#define MMI_CMD2_MEAS_SET_ADC_DELAY_MS        0x05
#define MMI_CMD3_MEAS_SET_ADC_DELAY_MS_CH1    0x01
#define MMI_CMD3_MEAS_SET_ADC_DELAY_MS_CH2    0x02
#define MMI_CMD3_MEAS_SET_ADC_DELAY_MS_CH3    0x03
#define MMI_CMD3_MEAS_SET_ADC_DELAY_MS_ALL    0x04
#define MMI_CMD3_MEAS_SET_ADC_DELAY_DATA_LEN (2)

#define MMI_CMD2_MEAS_SET_STABLE_TEMPERATURE                 0x06
#define MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE          0x01
#define MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE_DATA_LEN (2)

#define MMI_CMD2_MEAS_SET_CH_TEST_W_RESP                     0x07
#define MMI_CMD3_MEAS_SET_CH_TEST_CH1                        0x01
#define MMI_CMD3_MEAS_SET_CH_TEST_CH2                        0x02
#define MMI_CMD3_MEAS_SET_CH_TEST_CH3                        0x03
#define MMI_CMD3_MEAS_SET_CH_TEST_DATA_LEN                   (2)
#define MMI_CMD2_MEAS_SET_RESP_CH_TEST                       0x17

#define MMI_CMD2_MEAS_SET_TEMPERATURE_OFFSET                 0x08
#define MMI_CMD3_MEAS_SET_TEMPERATURE_OFFSET_DEGREE          0x00
#define MMI_CMD3_MEAS_SET_TEMPERATURE_OFFSET_DEGREE_DATA_LEN (2)

/* RX Command Group 3: Request Measuring */
#define MMI_CMD1_MEAS_REQ                0x0B
#define MMI_CMD1_MEAS_REQ_RESP           0x1B
#define MMI_CMD1_MEAS_REQ_MAX_DATA_LEN   MMI_CMD3_MEAS_REQ_ALL_DATA_LEN

#define MMI_CMD2_MEAS_REQ_ALL_W_RESP     0x00
#define MMI_CMD3_MEAS_REQ_ALL            0x00
#define MMI_CMD3_MEAS_REQ_ALL_DATA_LEN   (18)

#define MMI_CMD3_MEAS_REQ_ADC_MIN_DATA_LEN (2)
#define MMI_CMD3_MEAS_REQ_ADC_MAX_DATA_LEN (6)

#define MMI_CMD2_MEAS_REQ_TEMPERATURE_W_RESP  0x01
#define MMI_CMD3_MEAS_REQ_TEMPERATURE_CH1     0x01
#define MMI_CMD3_MEAS_REQ_TEMPERATURE_CH2     0x02
#define MMI_CMD3_MEAS_REQ_TEMPERATURE_CH3     0x03
#define MMI_CMD3_MEAS_REQ_TEMPERATURE_ALL     0x04

#define MMI_CMD2_MEAS_REQ_RESP_ADC_W_RESP     0x02
#define MMI_CMD3_MEAS_REQ_PD_ADC_CH1          0x01
#define MMI_CMD3_MEAS_REQ_PD_ADC_CH2          0x02
#define MMI_CMD3_MEAS_REQ_PD_ADC_CH3          0x03
#define MMI_CMD3_MEAS_REQ_PD_ADC_ALL          0x04

#define MMI_CMD2_MEAS_REQ_MONITOR_ADC_W_RESP  0x03
#define MMI_CMD3_MEAS_REQ_MONITOR_ADC_CH1     0x01
#define MMI_CMD3_MEAS_REQ_MONITOR_ADC_CH2     0x02
#define MMI_CMD3_MEAS_REQ_MONITOR_ADC_CH3     0x03
#define MMI_CMD3_MEAS_REQ_MONITOR_ADC_ALL     0x04

/* RX Command Group 4: Request Device Status */
#define MMI_CMD1_REQ_DEVICE_STATUS          0x0C
#define MMI_CMD1_REQ_DEVICE_STATUS_RESP     0x1C
#define MMI_CMD2_REQ_DEVICE_STATUS_W_RESP   0x00
#define MMI_CMD3_REQ_DEVICE_STATUS          0x00
#define MMI_CMD3_REQ_DEVICE_STATUS_DATA_LEN (4)

/* RX Command Group 5: Device Control */
#define MMI_CMD1_CTRL_DEVICE                  0x0D
#define MMI_CMD1_CTRL_DEVICE_MAX_DATA_LEN     (1)

#define MMI_CMD2_CTRL_DEVICE_LED              0x01
#define MMI_CMD3_CTRL_DEVICE_LED_CH1          0x01
#define MMI_CMD3_CTRL_DEVICE_LED_CH2          0x02
#define MMI_CMD3_CTRL_DEVICE_LED_CH3          0x03
#define MMI_CMD3_CTRL_DEVICE_LED_ALL          0x04
#define MMI_CMD3_CTRL_DEVICE_LED_DATA_LEN     (1)

#define MMI_CMD2_CTRL_DATA_SEND_MODE          0x02
#define MMI_CMD3_CTRL_DATA_SEND_MODE          0x00
#define MMI_CMD3_CTRL_DATA_SEND_MODE_DATA_LEN (1)

typedef union {
    uint8_t bVal[4];
    float fVal;
} FtoB;

/* Channel index data for measure */
typedef enum {
    CH1_IDX = 0,
    CH2_IDX,
    CH3_IDX,
    CH_NUM,
} MeasCh_t;

typedef enum {
    MEAS_SET_CAT_TEMP_ON_OFF = 0,
    MEAS_SET_CAT_LED_ON_TIME,
    MEAS_SET_CAT_LED_ON_LEVEL,
    MEAS_SET_CAT_ADC_SAMPLE_CNT,
    MEAS_SET_CAT_ADC_ON_DELAY,
    MEAS_SET_CAT_STABLE_TEMPERATURE,
    MEAS_SET_CAT_CH_TEST,
    MEAS_SET_CAT_TEMPERATURE_OFFSET,
    MEAS_SET_CAT_MAX = MEAS_SET_CAT_TEMPERATURE_OFFSET,
} MeasSetCat_t;

typedef enum {
    MEAS_RESULT_CAT_TEMPERATURE = 0,
    MEAS_RESULT_CAT_RECV_PD_ADC,
    MEAS_RESULT_CAT_MONITOR_PD_ADC,
    MEAS_RESULT_CAT_MAX = MEAS_RESULT_CAT_MONITOR_PD_ADC,
} MeasResultCat_t;

typedef enum {
    MEAS_SET_CH_1 = 0x01,
    MEAS_SET_CH_2 = 0x02,
    MEAS_SET_CH_3 = 0x03,
    MEAS_SET_CH_ALL = 0x04,
    MEAS_SET_CH_MAX = MEAS_SET_CH_ALL,
} MeasSetChVal_t;

typedef enum {
    TEMP_CTRL_FORCE_OFF = 0x00,
    TEMP_CTRL_AUTO = 0x01,
    TEMP_CTRL_FORCE_ON = 0x02, /* for test */
} MeasSetTempCtrlType_t;

typedef enum {
    CH_TEST_OFF = 0x00,
    CH_TEST_ON = 0x01,
} MeasSetChTest_t;

typedef enum {
    LED_CTRL_FORCE_OFF = 0x00,
    LED_CTRL_FORCE_ON = 0x01,
    LED_CTRL_AUTO = 0x02,
} MeasCtrlLedType_t;

typedef enum {
    MEAS_OP_MODE_SINGLE = 0x00,
    MEAS_OP_MODE_CONTINUOUS = 0x01, /* Default */
    MEAS_OP_MODE_MAX,
} MeasCtrlOpMode_t;

/* Default Measure Setting */
#define MEAS_SET_DEFAULT_TEMP_CTRL_TYPE            LED_CTRL_AUTO

#define MEAS_SET_MIN_LED_ON_TIME_MS                100
#define MEAS_SET_MAX_LED_ON_TIME_MS                1000
#define MEAS_SET_DEFAULT_LED_ON_TIME_MS            200

#define MEAS_SET_MIN_LED_ON_LEVEL                  0
#define MEAS_SET_MAX_LED_ON_LEVEL                  0xFFF
#define MEAS_SET_DEFAULT_LED_ON_LEVEL              0xFFF

#define MEAS_SET_MIN_ADC_SAMPLE_CNT                10
#define MEAS_SET_MAX_ADC_SAMPLE_CNT                100
#define MEAS_SET_DEFAULT_ADC_SAMPLE_CNT            20

#define MEAS_SET_MIN_ADC_DELAY_MS                  1
#define MEAS_SET_MAX_ADC_DELAY_MS                  100
#define MEAS_SET_DEFAULT_ADC_DELAY_MS              (MEAS_SET_DEFAULT_LED_ON_TIME_MS / MEAS_SET_DEFAULT_ADC_SAMPLE_CNT)

#define MEAS_SET_STABLE_TEMPERATURE_MIN_DEGREE     20
#define MEAS_SET_STABLE_TEMPERATURE_MAX_DEGREE     45
#define MEAS_SET_DEFAULT_STABLE_TEMPERATURE_DEGREE 35

#define MEAS_SET_TEMPERATURE_OFFSET_MIN_DEGREE     0
#define MEAS_SET_TEMPERATURE_OFFSET_MAX_DEGREE     50
#define MEAS_SET_DEFAULT_TEMPERATURE_OFFSET_DEGREE 30
#define MEAS_SET_TEMPERATURE_DEGREE_SCALE          100

#define MEAS_SET_DEFAULT_LED_CTRL_TYPE             LED_CTRL_AUTO

#pragma pack(push, 1)
typedef struct {
    MeasSetTempCtrlType_t temp_ctrl_type[CH_NUM]; /* Ch1 - CH3 */
    uint16_t led_on_time[CH_NUM];
    uint16_t led_on_level[CH_NUM];
    uint16_t adc_sample_cnt[CH_NUM];
    uint16_t adc_delay_ms[CH_NUM];
    uint16_t stable_temperature;
    uint16_t temperature_offset[CH_NUM];
    /* ~NVM */
    MeasSetChTest_t ch_test[CH_NUM];
    MeasCtrlLedType_t led_ctrl_state[CH_NUM]; /* Ch1 - CH3 */
} MeasSetData_t;
#pragma pack(pop)

typedef struct {
    union {
        MeasSetData_t settings;
        uint8_t msg[MMI_CMD3_MEAS_SET_VAL_REQ_DATA_LEN + 3];
    };
} MeasSetDataMsg_t;

typedef struct {
    int16_t temperature_data[CH_NUM];
    int16_t recv_pd_data[CH_NUM];
    int16_t monitor_pd_data[CH_NUM];
} MeasResultData_t;

#define MEAS_TARGET_CH_DEACTIV 0
#define MEAS_TARGET_CH_ACTIVE  1
#pragma pack(push, 1)
typedef struct {
    bool led_on_status[CH_NUM];
    uint8_t target_ch[CH_NUM];
} MeasReqStatus_t;
#pragma pack(pop)

typedef struct {
    union {
        MeasReqStatus_t status;
        uint8_t msg[MMI_CMD3_REQ_DEVICE_STATUS_DATA_LEN];
    };
} MeasReqStatusMsg_t;

/* FRAM Address */
#define FRAM_TEMP_SETTING_SINGLE_DATA_LEN        MMI_CMD3_MEAS_SET_TEMP_ONOFF_DATA_LEN
#define FRAM_TEMP_SETTING_ADDR_CH1               0x00
#define FRAM_TEMP_SETTING_ADDR_CH2               FRAM_TEMP_SETTING_ADDR_CH1 + MMI_CMD3_MEAS_SET_TEMP_ONOFF_DATA_LEN
#define FRAM_TEMP_SETTING_ADDR_CH3               FRAM_TEMP_SETTING_ADDR_CH2 + MMI_CMD3_MEAS_SET_TEMP_ONOFF_DATA_LEN
#define FRAM_TEMP_SETTING_DATA_LEN               (MMI_CMD3_MEAS_SET_TEMP_ONOFF_DATA_LEN * 3)

#define FRAM_LED_ON_TIME_SINGLE_DATA_LEN         MMI_CMD3_MEAS_SET_LED_ON_TIME_DATA_LEN
#define FRAM_LED_ON_TIME_ADDR_CH1                (FRAM_TEMP_SETTING_ADDR_CH1 + FRAM_TEMP_SETTING_DATA_LEN)
#define FRAM_LED_ON_TIME_ADDR_CH2                FRAM_LED_ON_TIME_ADDR_CH1 + FRAM_LED_ON_TIME_SINGLE_DATA_LEN
#define FRAM_LED_ON_TIME_ADDR_CH3                FRAM_LED_ON_TIME_ADDR_CH2 + FRAM_LED_ON_TIME_SINGLE_DATA_LEN
#define FRAM_LED_ON_TIME_DATA_LEN                (MMI_CMD3_MEAS_SET_LED_ON_TIME_DATA_LEN * 3)

#define FRAM_LED_ON_LEVEL_SINGLE_DATA_LEN        MMI_CMD3_MEAS_SET_LED_LEVEL_DATA_LEN
#define FRAM_LED_ON_LEVEL_ADDR_CH1               (FRAM_LED_ON_TIME_ADDR_CH1 + FRAM_LED_ON_TIME_DATA_LEN)
#define FRAM_LED_ON_LEVEL_ADDR_CH2               FRAM_LED_ON_LEVEL_ADDR_CH1 + FRAM_LED_ON_LEVEL_SINGLE_DATA_LEN
#define FRAM_LED_ON_LEVEL_ADDR_CH3               FRAM_LED_ON_LEVEL_ADDR_CH2 + FRAM_LED_ON_LEVEL_SINGLE_DATA_LEN
#define FRAM_LED_ON_LEVEL_DATA_LEN               (MMI_CMD3_MEAS_SET_LED_LEVEL_DATA_LEN * 3)

#define FRAM_ADC_SAMPLE_CNT_SINGLE_DATA_LEN      MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_DATA_LEN
#define FRAM_ADC_SAMPLE_CNT_ADDR_CH1             (FRAM_LED_ON_LEVEL_ADDR_CH1 + FRAM_LED_ON_LEVEL_DATA_LEN)
#define FRAM_ADC_SAMPLE_CNT_ADDR_CH2             FRAM_ADC_SAMPLE_CNT_ADDR_CH1 + FRAM_ADC_SAMPLE_CNT_SINGLE_DATA_LEN
#define FRAM_ADC_SAMPLE_CNT_ADDR_CH3             FRAM_ADC_SAMPLE_CNT_ADDR_CH2 + FRAM_ADC_SAMPLE_CNT_SINGLE_DATA_LEN
#define FRAM_ADC_SAMPLE_CNT_DATA_LEN             (FRAM_ADC_SAMPLE_CNT_SINGLE_DATA_LEN * 3)

#define FRAM_ADC_DELAY_MS_SINGLE_DATA_LEN        MMI_CMD3_MEAS_SET_ADC_DELAY_DATA_LEN
#define FRAM_ADC_DELAY_MS_ADDR_CH1               (FRAM_ADC_SAMPLE_CNT_ADDR_CH1 + FRAM_ADC_SAMPLE_CNT_DATA_LEN)
#define FRAM_ADC_DELAY_MS_ADDR_CH2               FRAM_ADC_DELAY_MS_ADDR_CH1 + FRAM_ADC_DELAY_MS_SINGLE_DATA_LEN
#define FRAM_ADC_DELAY_MS_ADDR_CH3               FRAM_ADC_DELAY_MS_ADDR_CH2 + FRAM_ADC_DELAY_MS_SINGLE_DATA_LEN
#define FRAM_ADC_DELAY_MS_DATA_LEN               (FRAM_ADC_DELAY_MS_SINGLE_DATA_LEN * 3)

#define FRAM_STABLE_TEMPERATURE_SINGLE_DATA_LEN  MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE_DATA_LEN
#define FRAM_STABLE_TEMPERATURE_ADDR             (FRAM_ADC_DELAY_MS_ADDR_CH1 + FRAM_ADC_DELAY_MS_DATA_LEN)
#define FRAM_STABLE_TEMPERATURE_DATA_LEN         FRAM_STABLE_TEMPERATURE_SINGLE_DATA_LEN

#define FRAM_TEMPERATURE_OFFSET_SINGLE_DATA_LEN  MMI_CMD3_MEAS_SET_TEMPERATURE_OFFSET_DEGREE_DATA_LEN
#define FRAM_TEMPERATURE_OFFSET_ADDR_CH1         (FRAM_STABLE_TEMPERATURE_ADDR + FRAM_STABLE_TEMPERATURE_DATA_LEN)
#define FRAM_TEMPERATURE_OFFSET_ADDR_CH2         FRAM_TEMPERATURE_OFFSET_ADDR_CH1 + FRAM_TEMPERATURE_OFFSET_SINGLE_DATA_LEN
#define FRAM_TEMPERATURE_OFFSET_ADDR_CH3         FRAM_TEMPERATURE_OFFSET_ADDR_CH2 + FRAM_TEMPERATURE_OFFSET_SINGLE_DATA_LEN
#define FRAM_TEMPERATURE_OFFSET_DATA_LEN         (FRAM_RECV_PD_RESULT_SINGLE_DATA_LEN * 3)

#define FRAM_TEMP_RESULT_SINGLE_DATA_LEN         MMI_CMD3_MEAS_REQ_ADC_MIN_DATA_LEN
#define FRAM_TEMP_RESULT_ADDR_CH1                (FRAM_TEMPERATURE_OFFSET_ADDR_CH1 + FRAM_TEMPERATURE_OFFSET_DATA_LEN)
#define FRAM_TEMP_RESULT_ADDR_CH2                FRAM_TEMP_RESULT_ADDR_CH1 + FRAM_TEMP_RESULT_SINGLE_DATA_LEN
#define FRAM_TEMP_RESULT_ADDR_CH3                FRAM_TEMP_RESULT_ADDR_CH2 + FRAM_TEMP_RESULT_SINGLE_DATA_LEN
#define FRAM_TEMP_RESULT_DATA_LEN                (FRAM_TEMP_RESULT_SINGLE_DATA_LEN * 3)

#define FRAM_RECV_PD_RESULT_SINGLE_DATA_LEN      MMI_CMD3_MEAS_REQ_ADC_MIN_DATA_LEN
#define FRAM_RECV_PD_RESULT_ADDR_CH1             (FRAM_TEMP_RESULT_ADDR_CH1 + FRAM_TEMP_RESULT_DATA_LEN)
#define FRAM_RECV_PD_RESULT_ADDR_CH2             FRAM_RECV_PD_RESULT_ADDR_CH1 + FRAM_RECV_PD_RESULT_SINGLE_DATA_LEN
#define FRAM_RECV_PD_RESULT_ADDR_CH3             FRAM_RECV_PD_RESULT_ADDR_CH2 + FRAM_RECV_PD_RESULT_SINGLE_DATA_LEN
#define FRAM_RECV_PD_RESULT_DATA_LEN             (FRAM_RECV_PD_RESULT_SINGLE_DATA_LEN * 3)

#define FRAM_MONITOR_PD_RESULT_SINGLE_DATA_LEN   MMI_CMD3_MEAS_REQ_ADC_MIN_DATA_LEN
#define FRAM_MONITOR_PD_RESULT_ADDR_CH1          (FRAM_RECV_PD_RESULT_ADDR_CH1 + FRAM_RECV_PD_RESULT_DATA_LEN)
#define FRAM_MONITOR_PD_RESULT_ADDR_CH2          FRAM_MONITOR_PD_RESULT_ADDR_CH1 + FRAM_MONITOR_PD_RESULT_SINGLE_DATA_LEN
#define FRAM_MONITOR_PD_RESULT_ADDR_CH3          FRAM_MONITOR_PD_RESULT_ADDR_CH2 + FRAM_MONITOR_PD_RESULT_SINGLE_DATA_LEN
#define FRAM_MONITOR_PD_RESULT_DATA_LEN          (FRAM_MONITOR_PD_RESULT_SINGLE_DATA_LEN * 3)

#define FRAM_DEV_CTRL_LED_STATUS_SINGLE_DATA_LEN MMI_CMD3_CTRL_DEVICE_LED_DATA_LEN
#define FRAM_DEV_CTRL_LED_STATUS_ADDR_CH1       (FRAM_MONITOR_PD_RESULT_ADDR_CH1 + FRAM_MONITOR_PD_RESULT_DATA_LEN)
#define FRAM_DEV_CTRL_LED_STATUS_ADDR_CH2        FRAM_DEV_CTRL_LED_STATUS_ADDR_CH1 + FRAM_DEV_CTRL_LED_STATUS_SINGLE_DATA_LEN
#define FRAM_DEV_CTRL_LED_STATUS_ADDR_CH3        FRAM_DEV_CTRL_LED_STATUS_ADDR_CH2 + FRAM_DEV_CTRL_LED_STATUS_SINGLE_DATA_LEN
#define FRAM_DEV_CTRL_LED_STATUS_DATA_LEN        FRAM_DEV_CTRL_LED_STATUS_SINGLE_DATA_LEN

/* FRAM Write/Read Max Len */
#define FRAM_DATA_MIN_ADDR                FRAM_TEMP_SETTING_ADDR_CH1
#define FRAM_DATA_MAX_ADDR                FRAM_DEV_CTRL_LED_STATUS_ADDR_CH3
#define FRAM_DATA_MAX_ADDR_DATA_LEN       FRAM_DEV_CTRL_LED_STATUS_DATA_LEN
#define FRAM_DATA_MAX_LEN                 (FRAM_DATA_MAX_ADDR + FRAM_DATA_MAX_ADDR_DATA_LEN)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _DATA_TYPE_H_ */
