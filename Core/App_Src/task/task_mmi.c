/**
 ******************************************************************************
 * @file           : task_mmi.c
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
#include "task_mmi.h"
#include "task_meas.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PROTOCOL_BUFF_TX_LEN_MAX	MMI_PROTOCOL_TX_MSG_LEN_MAX
#define PROTOCOL_BUFF_RX_LEN_MAX	MMI_PROTOCOL_RX_MSG_LEN_MAX

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef _mmi_send(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t data_len, uint8_t *p_data);
static void _protocol_decoder(uint8_t *p_org_line_arr, uint16_t arr_len);
static bool _protocol_chksum_check(uint8_t *arr, int cnt);
static HAL_StatusTypeDef _process_command(uint8_t *arr, uint16_t len);
static HAL_StatusTypeDef _process_get_device_info(uint8_t cmd2, uint8_t cmd3);
static HAL_StatusTypeDef _process_set_meas(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len);
static HAL_StatusTypeDef _process_req_meas(uint8_t cmd2, uint8_t cmd3);
static HAL_StatusTypeDef _process_get_device_status(uint8_t cmd2, uint8_t cmd3);
static HAL_StatusTypeDef _process_ctrl_device(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len);
/* Private variables ---------------------------------------------------------*/
uint8_t rx_msg_buff[PROTOCOL_BUFF_RX_LEN_MAX] = {
    0, };
uint8_t tx_msg_buff[PROTOCOL_BUFF_TX_LEN_MAX] = {
    0, };

/* Public user code ----------------------------------------------------------*/
void Task_MMI_Decoder(uint8_t *p_ch, uint16_t len) {
    static uint8_t one_line[PROTOCOL_BUFF_RX_LEN_MAX] = {
        0, };
    static uint8_t ch_cnt = 0;

    for (uint8_t idx = 0; idx < len; idx++) {
        one_line[ch_cnt++] = *(p_ch + idx);

        if (one_line[ch_cnt - 1] == MMI_PROTOCOL_ETX) {
            _protocol_decoder(&one_line[0], ch_cnt);
            ch_cnt = 0;

            memset(&one_line[0], 0, PROTOCOL_BUFF_RX_LEN_MAX);
        }

        if (ch_cnt + 1 >= PROTOCOL_BUFF_RX_LEN_MAX) {
            ch_cnt = 0;
        }
    }
}

HAL_StatusTypeDef Task_MMI_SendDeviceInfo(void) {
    SYS_VERIFY_SUCCESS(_process_get_device_info(MMI_CMD2_INFO_WHO_AM_I_AND_DEVICE_I_W_RESP, MMI_CMD2_INFO_WHO_AM_I_AND_DEVICE_I_W_RESP));

    return HAL_OK;
}

HAL_StatusTypeDef Task_MMI_SendMeasResult(void) {
    MeasSetChVal_t result_ch;
    MeasResultData_t result_data_buff;
    uint8_t temp_msg_buff[MMI_CMD3_MEAS_REQ_START_DATA_LEN];
    Task_Meas_RequestResult(&result_ch, &result_data_buff);

    temp_msg_buff[0] = result_ch;
    memcpy(&temp_msg_buff[1], &result_data_buff.temperature_data[result_ch], 2);
    memcpy(&temp_msg_buff[3], &result_data_buff.recv_pd_data[result_ch], 2);
    memcpy(&temp_msg_buff[5], &result_data_buff.monitor_pd_data[result_ch], 2);

    return _mmi_send(MMI_CMD1_MEAS_REQ, MMI_CMD2_MEAS_REQ_START_W_DELAYED_RESP, result_ch, MMI_CMD3_MEAS_REQ_START_DATA_LEN, &temp_msg_buff[0]);
}

/* Private user code ---------------------------------------------------------*/
static HAL_StatusTypeDef _mmi_send(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t data_len, uint8_t *p_data) {
    uint8_t orig_arr[PROTOCOL_BUFF_TX_LEN_MAX] = {
        0, };
    uint8_t chksum = MMI_PROTOCOL_CHKSUM_INIT;
    uint8_t chg_cnt = 0;
    uint8_t arr_idx = 1;

    orig_arr[MMI_PROTOCOL_IDX_STX] = MMI_PROTOCOL_STX;
    orig_arr[MMI_PROTOCOL_IDX_LEN] = (data_len >> 8) & 0xFF;
    orig_arr[MMI_PROTOCOL_IDX_LEN + 1] = (data_len) & 0xFF;
    orig_arr[MMI_PROTOCOL_IDX_CMD1] = cmd1;
    orig_arr[MMI_PROTOCOL_IDX_CMD2] = cmd2;
    orig_arr[MMI_PROTOCOL_IDX_CMD3] = cmd3;

    for (uint16_t i = 0; i < data_len; i++) {
        orig_arr[MMI_PROTOCOL_IDX_DATA + i] = p_data[i];
        chksum += p_data[i];
    }

    for (uint8_t i = 0; i < 6; i++) {
        chksum += orig_arr[i];
    }

    orig_arr[MMI_PROTOCOL_IDX_DATA + data_len] = chksum;
    orig_arr[MMI_PROTOCOL_IDX_DATA + data_len + 1] = MMI_PROTOCOL_ETX;

    /* Make bitstuffing array */
    tx_msg_buff[MMI_PROTOCOL_IDX_STX] = MMI_PROTOCOL_STX;

    for (arr_idx = 1; arr_idx < 6 + data_len + 1; arr_idx++) {
        switch (orig_arr[arr_idx]) {
            case 0xC0:
                tx_msg_buff[arr_idx + chg_cnt++] =
                MMI_PROTOCOL_BYTE_STUFF_PADDING;
                tx_msg_buff[arr_idx + chg_cnt] =
                MMI_PROTOCOL_BYTE_STUFF_C0_REPLACE;
                break;

            case 0xDB:
                tx_msg_buff[arr_idx + chg_cnt++] =
                MMI_PROTOCOL_BYTE_STUFF_PADDING;
                tx_msg_buff[arr_idx + chg_cnt] =
                MMI_PROTOCOL_BYTE_STUFF_DB_REPLACE;
                break;

            case 0xC2:
                tx_msg_buff[arr_idx + chg_cnt++] =
                MMI_PROTOCOL_BYTE_STUFF_PADDING;
                tx_msg_buff[arr_idx + chg_cnt] =
                MMI_PROTOCOL_BYTE_STUFF_C2_REPLACE;
                break;

            default:
                tx_msg_buff[arr_idx + chg_cnt] = orig_arr[arr_idx];
                break;
        }
    }

    tx_msg_buff[arr_idx + chg_cnt] = MMI_PROTOCOL_ETX;

    return UART_SendMMI(&tx_msg_buff[0], arr_idx + chg_cnt + 1);
}

static void _protocol_decoder(uint8_t *p_org_line_arr, uint16_t arr_len) {
    uint8_t bitsttf_line[512] = {
        0, };
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
                if (_protocol_chksum_check(bitsttf_line, arr_idx)) {
                    _process_command(bitsttf_line, arr_idx);
                    break;
                }
                else {
                    SYS_LOG_ERR("Checksum failed");
                    return;
                }

            default:
                bitsttf_line[arr_idx] = p_org_line_arr[arr_idx + chg_cnt];
                break;
        }
    }

    SYS_LOG_DEBUG("Bitstff Original: ");
    for (arr_idx = 0; arr_idx < arr_len; arr_idx++) {
        SYS_LOG_DEBUG("%02X", p_org_line_arr[arr_idx]);
    }

    SYS_LOG_DEBUG("Bitstff Changed : ");
    for (arr_idx = 0; arr_idx < arr_len + chg_cnt; arr_idx++) {
        SYS_LOG_DEBUG("%02X", bitsttf_line[arr_idx]);
    }
}

static bool _protocol_chksum_check(uint8_t *arr, int cnt) {
    uint32_t chksum_32 = MMI_PROTOCOL_CHKSUM_INIT;
    uint8_t chksum_8 = 0;

    for (int i = 0; i < cnt - 1; i++) {
        chksum_32 += arr[i];
    }

    chksum_8 = (uint8_t) chksum_32;

    if (chksum_8 != arr[cnt - 1]) {
        return false;
    }

    return true;
}

static HAL_StatusTypeDef _process_command(uint8_t *p_arr, uint16_t len) {
    memcpy(rx_msg_buff, p_arr, len);
    uint8_t cmd1 = rx_msg_buff[MMI_PROTOCOL_IDX_CMD1];
    uint8_t cmd2 = rx_msg_buff[MMI_PROTOCOL_IDX_CMD2];
    uint8_t cmd3 = rx_msg_buff[MMI_PROTOCOL_IDX_CMD3];
    uint8_t data_len = (rx_msg_buff[MMI_PROTOCOL_IDX_LEN] << 8) | rx_msg_buff[MMI_PROTOCOL_IDX_LEN + 1];
    uint8_t *p_data = &rx_msg_buff[MMI_PROTOCOL_IDX_DATA];

    switch (cmd1) {
        case MMI_CMD1_INFO:
            SYS_LOG_INFO("[Command 1-1: %0x0X] Get device information", cmd1);
            _process_get_device_info(cmd2, cmd3);
            break;

        case MMI_CMD1_MEAS_SET:
            SYS_LOG_INFO("[Command 1-2: %0x0X] Set measure settings", cmd1);
            _process_set_meas(cmd2, cmd3, p_data, data_len);
            break;

        case MMI_CMD1_MEAS_REQ:
            SYS_LOG_INFO("[Command 1-3: %0x0X] Request measure", cmd1);
            _process_req_meas(cmd2, cmd3);
            break;

        case MMI_CMD1_REQ_DEVICE_STATUS:
            SYS_LOG_INFO("[Command 1-4: %0x0X] Request device status", cmd1);
            _process_get_device_status(cmd2, cmd3);
            break;

        case MMI_CMD1_CTRL_DEVICE:
            SYS_LOG_INFO("[Command 1-5: %0x0X] control device", cmd1);
            _process_ctrl_device(cmd2, cmd3, p_data, data_len);
            break;

        default:
            SYS_LOG_ERR("Invalid cmd2 value: %d", cmd2);
            return HAL_ERROR;
    }

    /* clean rx buffer */
    memset(rx_msg_buff, 0, sizeof(rx_msg_buff));

    return HAL_OK;
}

static HAL_StatusTypeDef _process_get_device_info(uint8_t cmd2, uint8_t cmd3) {
    uint8_t device_info_data_val[8] = {
        0, };
    uint8_t device_info_data_len = 0;
    
    switch (cmd2) {
        case MMI_CMD2_INFO_WHO_AM_I_AND_DEVICE_I_W_RESP:
            SYS_VERIFY_TRUE(MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE == cmd3);
            device_info_data_len = MMI_CMD3_INFO_WHO_AM_I_AND_DEVICE_DATA_LEN;
            device_info_data_val[0] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 3) & 0xFF;
            device_info_data_val[1] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 2) & 0xFF;
            device_info_data_val[2] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 1) & 0xFF;
            device_info_data_val[3] = MMI_PROTOCOL_WHO_AM_I_VAL & 0xFF;
            device_info_data_val[4] = SYS_FW_MAJOR_VER;
            device_info_data_val[5] = SYS_FW_MINOR_VER;
            device_info_data_val[6] = (SYS_FW_PATCH_VER >> 1) & 0xFF;
            device_info_data_val[7] = SYS_FW_PATCH_VER & 0xFF;
            break;

        case MMI_CMD2_INFO_WHO_AM_I_W_RESP:
            SYS_VERIFY_TRUE(MMI_CMD3_INFO_WHO_AM_I == cmd3);
            device_info_data_len = MMI_CMD3_INFO_WHO_AM_I_DATA_LEN;
            device_info_data_val[0] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 3) & 0xFF;
            device_info_data_val[1] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 2) & 0xFF;
            device_info_data_val[2] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 1) & 0xFF;
            device_info_data_val[3] = MMI_PROTOCOL_WHO_AM_I_VAL & 0xFF;
            break;

        case MMI_CMD2_INFO_DEVICE_I_W_RESP:
            SYS_VERIFY_TRUE(MMI_CMD3_INFO_DEVICE == cmd3);
            device_info_data_len = MMI_CMD3_INFO_DEVICE_DATA_LEN;
            device_info_data_val[0] = SYS_FW_MAJOR_VER;
            device_info_data_val[1] = SYS_FW_MINOR_VER;
            device_info_data_val[2] = (SYS_FW_PATCH_VER >> 1) & 0xFF;
            device_info_data_val[3] = SYS_FW_PATCH_VER & 0xFF;
            break;

        default:
            SYS_LOG_ERR("Invalid cmd2 value: %d", cmd2);
            return HAL_ERROR;
    }

    SYS_VERIFY_SUCCESS(_mmi_send(MMI_CMD1_INFO, cmd2, cmd3, device_info_data_len, &device_info_data_val[0]));

    return HAL_OK;
}

static HAL_StatusTypeDef _process_set_meas(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len) {
    if (MMI_CMD2_MEAS_SET_VAL_REQ_W_RESP == cmd2) {
        MeasSetDataMsg_t set_data_buff;

        SYS_VERIFY_TRUE(MMI_CMD3_MEAS_SET_VAL_REQ == cmd3);

        Task_Meas_Get_Set(&set_data_buff.settings);
        SYS_VERIFY_SUCCESS(_mmi_send(MMI_CMD1_MEAS_SET, cmd2, cmd3, MMI_CMD3_MEAS_SET_VAL_REQ_DATA_LEN, &set_data_buff.msg[0]));

        return HAL_OK;
    }
    else {
        MeasSetChVal_t ch_cfg = (MeasSetChVal_t) cmd3; /* cmd3: ch select */
        uint8_t set_data_val[MMI_CMD1_MEAS_SET_MAX_DATA_LEN];

        memcpy(set_data_val, p_data, (data_len > sizeof(set_data_val) ? sizeof(set_data_val) : data_len));

        switch (cmd2) {
            case MMI_CMD2_MEAS_SET_TEMP:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_TEMP_ONOFF_DATA_LEN);
                return Task_Meas_Apply_Set(MEAS_SET_CAT_TEMP_ON_OFF, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_LED_ON_TIME:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_LED_ON_TIME_DATA_LEN);
                return Task_Meas_Apply_Set(MEAS_SET_CAT_LED_ON_TIME, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_LED_LEVEL:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_LED_LEVEL_DATA_LEN);
                return Task_Meas_Apply_Set(MEAS_SET_CAT_LED_ON_LEVEL, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_ADC_SAMPLE_CNT:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_DATA_LEN);
                return Task_Meas_Apply_Set(MEAS_SET_CAT_ADC_SAMPLE_CNT, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_ADC_DELAY_MS:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_ADC_DELAY_DATA_LEN);
                return Task_Meas_Apply_Set(MEAS_SET_CAT_ADC_ON_DELAY, ch_cfg, set_data_val);

            default:
                SYS_LOG_ERR("Invalid cmd2 value: %d", cmd2);
                return HAL_ERROR;

        }
    }
}

static HAL_StatusTypeDef _process_req_meas(uint8_t cmd2, uint8_t cmd3) {
    if (MMI_CMD2_MEAS_REQ_ALL_W_RESP == cmd2) {
        MeasResultAllDataMsg_t result_all_data_buff;

        SYS_VERIFY_TRUE(MMI_CMD3_MEAS_REQ_ALL == cmd3);
        SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_TEMP_ADC, MEAS_SET_CH_ALL, &result_all_data_buff.results.temperature_data[0]));
        SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_RECV_PD_ADC, MEAS_SET_CH_ALL, &result_all_data_buff.results.recv_pd_data[0]));
        SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_MONITOR_PD_ADC, MEAS_SET_CH_ALL, &result_all_data_buff.results.monitor_pd_data[0]));
        return _mmi_send(MMI_CMD1_MEAS_REQ, cmd2, cmd3, MMI_CMD3_MEAS_REQ_ALL_DATA_LEN, &result_all_data_buff.msg[0]);
    }
    else if (MMI_CMD2_MEAS_REQ_START_W_DELAYED_RESP == cmd2) {
        MeasSetChVal_t ch_cfg = (MeasSetChVal_t) cmd3; /* cmd3: ch select */
        return Task_Meas_Request(ch_cfg);
    }
    else {
        MeasSetChVal_t ch_cfg = (MeasSetChVal_t) cmd3; /* cmd3: ch select */
        MeasResultData_t result_data_val;
        uint8_t result_data_len = (MEAS_SET_CH_ALL == ch_cfg) ? MMI_CMD3_MEAS_REQ_ADC_MAX_DATA_LEN : MMI_CMD3_MEAS_REQ_ADC_MIN_DATA_LEN;

        switch (cmd2) {
            case MMI_CMD2_MEAS_REQ_TEMP_ADC_W_RESP:
                SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_TEMP_ADC, ch_cfg, &result_data_val.temperature_data[0]));
                return _mmi_send(MMI_CMD1_MEAS_REQ, cmd2, cmd3, result_data_len, (uint8_t*) &result_data_val.temperature_data[0]);

            case MMI_CMD2_MEAS_REQ_RESP_ADC_W_RESP:
                SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_RECV_PD_ADC, ch_cfg, &result_data_val.recv_pd_data[0]));
                return _mmi_send(MMI_CMD1_MEAS_REQ, cmd2, cmd3, result_data_len, (uint8_t*) &result_data_val.recv_pd_data[0]);

            case MMI_CMD2_MEAS_REQ_MONITOR_ADC_W_RESP:
                SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_MONITOR_PD_ADC, ch_cfg, &result_data_val.monitor_pd_data[0]));
                return _mmi_send(MMI_CMD1_MEAS_REQ, cmd2, cmd3, result_data_len, (uint8_t*) &result_data_val.monitor_pd_data[0]);

            default:
                SYS_LOG_ERR("Invalid cmd2 value: %d", cmd2);
                return HAL_ERROR;
        }
    }
}

static HAL_StatusTypeDef _process_get_device_status(uint8_t cmd2, uint8_t cmd3) {
    MeasReqStatusMsg_t status_data_buff;

    SYS_VERIFY_TRUE(MMI_CMD2_REQ_DEVICE_STATUS_W_RESP == cmd2);
    SYS_VERIFY_TRUE(MMI_CMD3_REQ_DEVICE_STATUS == cmd3);
    
    SYS_VERIFY_SUCCESS(Task_Meas_Get_Status(&status_data_buff.status));
    
    return _mmi_send(MMI_CMD1_REQ_DEVICE_STATUS, cmd2, cmd3, MMI_CMD3_MEAS_REQ_ALL_DATA_LEN, &status_data_buff.msg[0]);
}

static HAL_StatusTypeDef _process_ctrl_device(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len) {
    MeasSetChVal_t ch_cfg = (MeasSetChVal_t) cmd3; /* cmd3: ch select */
    uint8_t set_data_val[MMI_CMD1_CTRL_DEVICE_MAX_DATA_LEN];

    SYS_VERIFY_TRUE(data_len <= MMI_CMD1_CTRL_DEVICE_MAX_DATA_LEN);
    memcpy(set_data_val, p_data, data_len);

    switch (cmd2) {
        case MMI_CMD2_CTRL_DEVICE_LED:
            SYS_VERIFY_TRUE(data_len == MMI_CMD3_CTRL_DEVICE_LED_DATA_LEN);
            Task_Meas_Ctrl_Led(ch_cfg, set_data_val);
            break;

        case MMI_CMD2_CTRL_DEVICE_MONITOR:
            SYS_VERIFY_TRUE(data_len == MMI_CMD3_CTRL_DEVICE_MONITOR_LEN);
            Task_Meas_Ctrl_Monitor(ch_cfg, set_data_val);
            break;

        default:
            return HAL_ERROR;
            break;
    }
    
    return HAL_OK;
}

