/**
 ******************************************************************************
 * @file           : user_mmi.c
 * @brief          :
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "user_mmi.h"
#include "task_meas.h"

extern MeasAvrReq_t meas_cfg;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MMI_PROTOCOL_TX_MAX_LEN	128
#define MMI_PROTOCOL_RX_MAX_LEN	128
/* Private macro -------------------------------------------------------------*/
#define MMI_PROTOCOL_STX  					 0xC0
#define MMI_PROTOCOL_SPC  					 0xDB
#define MMI_PROTOCOL_ETX  					 0xC2
#define MMI_PROTOCOL_CHKSUM_INIT			 0xC2

#define MMI_PROTOCOL_BYTE_STUFF_PADDING     0xDB
#define MMI_PROTOCOL_BYTE_STUFF_C0_REPLACE  0xDC
#define MMI_PROTOCOL_BYTE_STUFF_DB_REPLACE  0xDD
#define MMI_PROTOCOL_BYTE_STUFF_C2_REPLACE  0xDE

/* Protocol Index */
/*
 *  []  [][]  []   []   []   []   ... []  []     []
 *  STX LEN   CMD1 CMD2 CMD3 Data N-Byte  ChkSum ETX
 *  	|
 *  	Data Byte length
 * */
#define MMI_PROTOCOL_STX_IDX		0
#define MMI_PROTOCOL_LEN_IDX		1
#define MMI_PROTOCOL_CMD1_IDX		3
#define MMI_PROTOCOL_CMD2_IDX		4
#define MMI_PROTOCOL_CMD3_IDX		5
#define MMI_PROTOCOL_DATA_IDX		6

/* Protocol Command */
#define CMD_1_DEV_INFO		 		0x55
#define CMD_2_WHO_AM_I		 	0xAA

#define CMD_1_DEV_ADC_SET			0x01
#define CMD_2_LED_SET  	 	    0x01
#define CMD_3_LED_SET_W	    0x01
#define CMD_3_LED_SET_R     0x02
#define CMD_2_ADC_SET			0x02
#define CMD_3_ADC_CFG_W		0x01
#define CMD_3_ADC_CFG_R     0x02
#define CMD_2_MUX_SET			0x03
#define CMD_3_M_CH1		    0x01
#define CMD_3_M_CH2		    0x02
#define CMD_3_M_CH3		    0x03
#define CMD_3_M_CH4		    0x04
#define CMD_3_M_ENABLE	    0x05
#define CMD_3_M_DISABLE	    0x06

#define CMD_1_DEV_MEAS_SET		 	0x02
#define CMD_2_MEAS_REQ_SET   	0x01
#define CMD_3_M1_REQ		0x01
#define CMD_3_M3_REQ		0x02

#define CMD_1_DEV_SEND_RESULT		0x03
#define CMD_2_RESULT_ADC		0x01
#define CMD_3_M1_RESULT		0x01
#define CMD_3_M3_RESULT		0x02

/* Private function prototypes -----------------------------------------------*/
static void _decoder(uint8_t *p_org_line_arr, uint16_t arr_len);
static int _checkChksum(uint8_t *arr, int cnt);
static void _processCommand(uint8_t *arr, uint16_t cnt);
static void _procDevInfo(uint8_t *p_arr);
static void _procDevAdcSet(uint8_t *p_arr);
static void _procDevMeasSet(uint8_t *p_arr);

/* Private variables ---------------------------------------------------------*/
uint8_t uart_send_buff[MMI_PROTOCOL_TX_MAX_LEN] = { 0, };

/* Public user code ----------------------------------------------------------*/
void MMI_Decoder(uint8_t *p_ch, uint16_t len) {
    static uint8_t one_line[MMI_PROTOCOL_RX_MAX_LEN] = { 0, };
    static uint8_t ch_cnt = 0;

    for (uint8_t idx = 0; idx < len; idx++) {
        one_line[ch_cnt++] = *(p_ch + idx);

        if (one_line[ch_cnt - 1] == MMI_PROTOCOL_ETX) {
            _decoder(&one_line[0], ch_cnt);
            ch_cnt = 0;

            memset(&one_line[0], 0, MMI_PROTOCOL_RX_MAX_LEN);
        }

        if (ch_cnt + 1 >= MMI_PROTOCOL_RX_MAX_LEN) {
            ch_cnt = 0;
        }
    }
}

void _decoder(uint8_t *p_org_line_arr, uint16_t arr_len) {
    uint8_t bitsttf_line[512] = { 0, };
    uint8_t arr_idx = 0;
    uint8_t chg_cnt = 0;

    for (uint8_t arr_idx = 0; arr_idx < arr_len; arr_idx++) {
        switch (p_org_line_arr[arr_idx + chg_cnt]) {
            case MMI_PROTOCOL_STX:
                bitsttf_line[arr_idx] = p_org_line_arr[arr_idx + chg_cnt];
                break;

            case MMI_PROTOCOL_SPC:
                chg_cnt++;
                switch (p_org_line_arr[arr_idx + chg_cnt]) {
                    case 0xDC:
                        bitsttf_line[arr_idx] = 0xC0;
                        break;

                    case 0xDD:
                        bitsttf_line[arr_idx] = 0xDB;
                        break;

                    case 0xDE:
                        bitsttf_line[arr_idx] = 0xC2;
                        break;
                }
                break;

            case MMI_PROTOCOL_ETX:
                if (_checkChksum(bitsttf_line, arr_idx) == 0) {
                    _processCommand(bitsttf_line, arr_idx);
                }
                else {
                    LogError("Checksum failed");
                    _processCommand(bitsttf_line, arr_idx);
                }
                break;

            default:
                bitsttf_line[arr_idx] = p_org_line_arr[arr_idx + chg_cnt];
                break;
        }
    }

    LogDebug("Bitstff Original: ");
    for (arr_idx = 0; arr_idx < arr_len; arr_idx++) {
        LogDebug("%02X", p_org_line_arr[arr_idx]);
    }

    LogDebug("Bitstff Changed : ");
    for (arr_idx = 0; arr_idx < arr_len + chg_cnt; arr_idx++) {
        LogDebug("%02X", bitsttf_line[arr_idx]);
    }
}

void MMI_M1ResultSender(MeasM1Result_t *p_result, uint8_t result_num) {
    uint8_t result_buff[PROTOCOL_DATA_MAX_SIZE] = { 0, };

    for (uint8_t idx = 0; idx < result_num; idx++) {
        result_buff[idx * 5] = p_result[idx].led_ch;

        /* Save float data to 4 byte int array */
        FtoB f2b;
        f2b.fVal = p_result[idx].avr_adc;
        result_buff[idx * 5 + 1] = f2b.bVal[0];
        result_buff[idx * 5 + 2] = f2b.bVal[1];
        result_buff[idx * 5 + 3] = f2b.bVal[2];
        result_buff[idx * 5 + 4] = f2b.bVal[3];
    }

    MMI_Sender(CMD_1_DEV_SEND_RESULT, CMD_2_RESULT_ADC, CMD_3_M1_RESULT, (uint8_t*) &result_buff[0],
    LED_MEAS_M1_RESULT_DATA_SIZE * result_num);
}

void MMI_M3ResultSender(MeasM3Result_t *p_result) {
    uint8_t result_buff[PROTOCOL_DATA_MAX_SIZE] = { 0, };
    FtoB f2b;

    /* Save float data to 4 byte int array */
    f2b.fVal = p_result->avr_adc;
    result_buff[0] = f2b.bVal[0];
    result_buff[1] = f2b.bVal[1];
    result_buff[2] = f2b.bVal[2];
    result_buff[3] = f2b.bVal[3];

    MMI_Sender(CMD_1_DEV_SEND_RESULT, CMD_2_RESULT_ADC, CMD_3_M3_RESULT, (uint8_t*) &result_buff[0], LED_MEAS_M3_RESULT_DATA_SIZE);
}

void MMI_Sender(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t len) {
    uint8_t orig_arr[MMI_PROTOCOL_TX_MAX_LEN] = { 0, };
    uint8_t chksum = MMI_PROTOCOL_CHKSUM_INIT;
    uint8_t chg_cnt = 0;
    uint8_t arr_idx = 1;

    orig_arr[MMI_PROTOCOL_STX_IDX] = 0xC0;
    orig_arr[MMI_PROTOCOL_LEN_IDX] = (len >> 8) & 0xFF;
    orig_arr[MMI_PROTOCOL_LEN_IDX + 1] = (len) & 0xFF;
    orig_arr[MMI_PROTOCOL_CMD1_IDX] = cmd1;
    orig_arr[MMI_PROTOCOL_CMD2_IDX] = cmd2;
    orig_arr[MMI_PROTOCOL_CMD3_IDX] = cmd3;

    for (uint16_t i = 0; i < len; i++) {
        orig_arr[MMI_PROTOCOL_DATA_IDX + i] = p_data[i];
        chksum += p_data[i];
    }

    for (uint8_t i = 0; i < 6; i++) {
        chksum += orig_arr[i];
    }

    orig_arr[MMI_PROTOCOL_DATA_IDX + len] = chksum;
    orig_arr[MMI_PROTOCOL_DATA_IDX + len + 1] = MMI_PROTOCOL_ETX;

    /* Make bitstuffing array */
    uart_send_buff[MMI_PROTOCOL_STX_IDX] = MMI_PROTOCOL_STX;

    for (arr_idx = 1; arr_idx < 6 + len + 1; arr_idx++) {
        switch (orig_arr[arr_idx]) {
            case 0xC0:
                uart_send_buff[arr_idx + chg_cnt++] =
                MMI_PROTOCOL_BYTE_STUFF_PADDING;
                uart_send_buff[arr_idx + chg_cnt] =
                MMI_PROTOCOL_BYTE_STUFF_C0_REPLACE;
                break;

            case 0xDB:
                uart_send_buff[arr_idx + chg_cnt++] =
                MMI_PROTOCOL_BYTE_STUFF_PADDING;
                uart_send_buff[arr_idx + chg_cnt] =
                MMI_PROTOCOL_BYTE_STUFF_DB_REPLACE;
                break;

            case 0xC2:
                uart_send_buff[arr_idx + chg_cnt++] =
                MMI_PROTOCOL_BYTE_STUFF_PADDING;
                uart_send_buff[arr_idx + chg_cnt] =
                MMI_PROTOCOL_BYTE_STUFF_C2_REPLACE;
                break;

            default:
                uart_send_buff[arr_idx + chg_cnt] = orig_arr[arr_idx];
                break;
        }
    }

    uart_send_buff[arr_idx + chg_cnt] = MMI_PROTOCOL_ETX;

    (void) UART_SendMMI(&uart_send_buff[0], arr_idx + chg_cnt + 1);
}

/* Private user code ---------------------------------------------------------*/
static int _checkChksum(uint8_t *arr, int cnt) {
    uint32_t chksum_32 = MMI_PROTOCOL_CHKSUM_INIT;
    uint8_t chksum_8 = 0;

    for (int i = 0; i < cnt - 1; i++) {
        chksum_32 += arr[i];
    }

    chksum_8 = (uint8_t) chksum_32;

    if (chksum_8 != arr[cnt - 1]) {
        return 1;
    }

    return 0;
}

static void _procDevInfo(uint8_t *p_arr) {
    uint8_t cmd_2 = p_arr[MMI_PROTOCOL_CMD2_IDX];
    uint8_t data[8];
    uint8_t ch;
    uint8_t sps;
    uint16_t samples;
    uint16_t wait_time;

    switch (cmd_2) {
        case CMD_2_WHO_AM_I:
            ADC_GetSetting(&ch, &sps, &samples, &wait_time);

            /* Setting Values */
            data[0] = 0xA5;
            data[1] = ch;
            data[2] = (sps >> 8) & 0xFF;
            data[3] = sps & 0xFF;
            data[4] = (samples >> 8) & 0xFF;
            data[5] = samples & 0xFF;
            data[6] = (wait_time >> 8) & 0xFF;
            data[7] = wait_time & 0xFF;

            MMI_Sender(CMD_1_DEV_INFO, CMD_2_WHO_AM_I, 0x00, data, 8);
            break;

        default:
            break;
    }
}

static void _procDevAdcSet(uint8_t *p_arr) {
    uint8_t cmd_2 = p_arr[MMI_PROTOCOL_CMD2_IDX];
    uint8_t cmd_3 = p_arr[MMI_PROTOCOL_CMD3_IDX];
    uint8_t len = (p_arr[MMI_PROTOCOL_LEN_IDX] << 8) | p_arr[MMI_PROTOCOL_LEN_IDX + 1];
    uint8_t data[12];

    memcpy(data, p_arr + 6, 12); //두번째 인자 = 복사할 메모리를 가리키고 있는 포인터

    switch (cmd_2) {
        case CMD_2_LED_SET:
            if (cmd_3 == CMD_3_LED_SET_W) {
                /* LED Schedule settigs */
                MeasSchedule_t schedules[4];

                int cnt = len / 3; // 12/3 = 4
                // 0-631(3byte 012), 1-453(3byte 345), 2-542(3byte 678), 3-247(3byte 91011)
                for (int i = 0; i < cnt; i++) {
                    schedules[i].led_ch = data[i * 3];
                    schedules[i].led_data = (data[i * 3 + 1] << 8) | data[i * 3 + 2];
                }
                Task_Meas_SetSchedule(schedules, cnt);
            }
            else if (cmd_3 == CMD_3_LED_SET_R) {
                /* LED DAC settings READ */
                uint8_t DAC_DATA[2];
                DAC_CheckStatus(&DAC_DATA[0]);

                MMI_Sender(CMD_1_DEV_INFO, CMD_2_WHO_AM_I, 0x00, DAC_DATA, 2);

                //Task_Meas_SetAdc((MeasAvrReq_t *)&data[0]);
            }
            break;
        case CMD_2_ADC_SET:
            if (cmd_3 == CMD_3_ADC_CFG_W) {
                meas_cfg.ch = data[0]; //
                meas_cfg.sps = (data[1] << 8) | data[2];
                meas_cfg.adc_num = (data[3] << 8) | data[4];
                meas_cfg.wait_time = (data[5] << 8) | data[6];

                Task_Meas_CFG_Change(&meas_cfg);
                ADC_CFG_Change(&meas_cfg);

                if (meas_cfg.ch == 0) //ch1
                        {
                    HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
                    meas_cfg.Mux_ch = 1;
                }
                else if (meas_cfg.ch == 1) //ch2
                        {
                    HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
                    meas_cfg.Mux_ch = 2;
                }
                else if (meas_cfg.ch == 2) //ch3
                        {
                    HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_SET);
                    meas_cfg.Mux_ch = 3;
                }
                else if (meas_cfg.ch == 3) //ch4
                        {
                    //HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                    //HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_SET);
                    //HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_SET);
                    //meas_cfg.Mux_ch = 4;
                }
                else {

                }
                //Task_Meas_SetAdc((MeasAvrReq_t *)&data[0]);
                //Task_Meas_MEAS_M1_Start();
            }
            else if (cmd_3 == CMD_3_ADC_CFG_R) {
                uint8_t ADC_data[8];
                uint8_t ch;
                uint8_t sps;
                uint16_t samples;
                uint16_t wait_time;
                ADC_GetSetting(&ch, &sps, &samples, &wait_time);

                /* Setting Values */
                ADC_data[0] = 0xA5;
                ADC_data[1] = ch;
                ADC_data[2] = (sps >> 8) & 0xFF;
                ADC_data[3] = sps & 0xFF;
                ADC_data[4] = (samples >> 8) & 0xFF;
                ADC_data[5] = samples & 0xFF;
                ADC_data[6] = (wait_time >> 8) & 0xFF;
                ADC_data[7] = wait_time & 0xFF;

                MMI_Sender(CMD_1_DEV_INFO, CMD_2_WHO_AM_I, 0x00, ADC_data, 8);
            }
            break;
        case CMD_2_MUX_SET:
            if (cmd_3 == CMD_3_M_CH1) {
                HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
                meas_cfg.Mux_ch = 1;
            }
            else if (cmd_3 == CMD_3_M_CH2) {
                HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
                meas_cfg.Mux_ch = 2;
            }
            else if (cmd_3 == CMD_3_M_CH3) {
                HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_SET);
                meas_cfg.Mux_ch = 3;
            }
            else if (cmd_3 == CMD_3_M_CH4) {
                HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_SET);
            }
            else if (cmd_3 == CMD_3_M_ENABLE) {
                HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
            }
            else if (cmd_3 == CMD_3_M_DISABLE) {
                HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_RESET);
            }
            else {

            }
            break;

        default:
            break;
    }
}

static void _procDevMeasSet(uint8_t *p_arr) {
    uint8_t cmd_2 = p_arr[MMI_PROTOCOL_CMD2_IDX];
    uint8_t cmd_3 = p_arr[MMI_PROTOCOL_CMD3_IDX];

    switch (cmd_2) {
        case CMD_2_MEAS_REQ_SET:
            if (cmd_3 == CMD_3_M1_REQ) {
                Task_Meas_MEAS_M1_Start();
            }
            else if (cmd_3 == CMD_3_M3_REQ) {
                Task_Meas_MEAS_M3_Start();
            }
            break;

        default:
            break;
    }
}

static void _processCommand(uint8_t *p_arr, uint16_t cnt) {
    uint8_t cmd_1 = p_arr[MMI_PROTOCOL_CMD1_IDX];

    switch (cmd_1) {
        case CMD_1_DEV_INFO:
            _procDevInfo(p_arr);
            break;

        case CMD_1_DEV_ADC_SET:
            _procDevAdcSet(p_arr);
            break;

        case CMD_1_DEV_MEAS_SET:
            _procDevMeasSet(p_arr);
            break;

        case CMD_1_DEV_SEND_RESULT:
            break;

        default:
            break;
    }
}
