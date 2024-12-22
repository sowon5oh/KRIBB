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
#include "hal_drv_fram.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MMI_MSG_DEBUG_LOG 0

#define PROTOCOL_BUFF_TX_LEN_MAX	MMI_PROTOCOL_TX_MSG_LEN_MAX
#define PROTOCOL_BUFF_RX_LEN_MAX	MMI_PROTOCOL_RX_MSG_LEN_MAX

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef _mmi_send(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t data_len, uint8_t *p_data);
static void _protocol_decoder(uint8_t *p_org_line_arr, uint16_t arr_len);
static bool _protocol_chksum_check(uint8_t *arr, uint8_t cnt);
static HAL_StatusTypeDef _process_command(uint8_t *arr, uint16_t len);
static HAL_StatusTypeDef _process_get_device_info(uint8_t cmd2, uint8_t cmd3);
static HAL_StatusTypeDef _process_set_meas(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len);
static HAL_StatusTypeDef _process_req_meas(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len);
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
            SYS_LOG_INFO("Decoder Start");
#if (MMI_MSG_DEBUG_LOG == 1)
            for (uint8_t hex_len = 0; hex_len < ch_cnt; hex_len++) {
                SYS_LOG_DEBUG("%02X", one_line[hex_len]);
            }
#endif
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

HAL_StatusTypeDef Task_MMI_SendMeasContinousResult(void) {
    MeasResultData_t result_data_buff;
    uint8_t temp_msg_buff[MMI_CMD3_MEAS_REQ_CONTINUOS_RESP_DATA_LEN];

    uint8_t temp_data_len = sizeof(result_data_buff.temperature_data);
    uint8_t recv_pd_data_len = sizeof(result_data_buff.recv_pd_data);
    uint8_t monitor_pd_data_len = sizeof(result_data_buff.monitor_pd_data);
    SYS_VERIFY_TRUE(temp_data_len + recv_pd_data_len + monitor_pd_data_len <= MMI_CMD3_MEAS_REQ_CONTINUOS_RESP_DATA_LEN);

    Task_Meas_Get_AllChResult(&result_data_buff);

    memcpy(&temp_msg_buff[0], &result_data_buff.temperature_data, sizeof(result_data_buff.temperature_data));
    memcpy(&temp_msg_buff[6], &result_data_buff.recv_pd_data, sizeof(result_data_buff.recv_pd_data));
    memcpy(&temp_msg_buff[12], &result_data_buff.monitor_pd_data, sizeof(result_data_buff.monitor_pd_data));

    return _mmi_send(MMI_CMD1_MEAS_REQ_RESP, MMI_CMD2_MEAS_REQ_DATA_W_RESP, MMI_CMD3_MEAS_REQ_CONTINUOS_MODE, MMI_CMD3_MEAS_REQ_CONTINUOS_RESP_DATA_LEN, &temp_msg_buff[0]);
}

HAL_StatusTypeDef Task_MMI_SendMeasSigleResult(void) {
    MeasSetChVal_t meas_ch;
    uint16_t meas_cnt;
    int16_t recv_pd_data_buff[MEAS_SET_CH_NUM];
    int16_t temperature_data_buff[MEAS_SET_CH_NUM];
    uint8_t temp_msg_buff[MMI_CMD3_MEAS_REQ_SINGLE_RESP_CH_ALL_DATA_LEN] = {
        0, };

    SYS_VERIFY_TRUE(MMI_CMD3_MEAS_REQ_SINGLE_RESP_CH_ALL_DATA_LEN >= (sizeof(meas_ch) + sizeof(meas_cnt) + sizeof(recv_pd_data_buff[0]) + sizeof(temperature_data_buff[0])));

    Task_Meas_Get_SingleChResult(&meas_ch, &meas_cnt, &recv_pd_data_buff[0], &temperature_data_buff[0]);

    if (meas_ch < MEAS_SET_CH_ALL && meas_ch >= MEAS_SET_CH_1) {
        /* Single Data */
        temp_msg_buff[0] = meas_ch;
        memcpy(&temp_msg_buff[1], &meas_cnt, sizeof(meas_cnt));
        memcpy(&temp_msg_buff[3], &recv_pd_data_buff[0], 2);
        memcpy(&temp_msg_buff[5], &temperature_data_buff[0], 2);

        return _mmi_send(MMI_CMD1_MEAS_REQ_RESP, MMI_CMD2_MEAS_REQ_DATA_W_RESP, MMI_CMD3_MEAS_REQ_SINGLE_MODE, MMI_CMD3_MEAS_REQ_SINGLE_RESP_CH_X_DATA_LEN, &temp_msg_buff[0]);
    }
    else {
        /* All Data */
        temp_msg_buff[0] = MMI_CMD3_MEAS_REQ_SINGLE_CH_ALL_ID;
        memcpy(&temp_msg_buff[1], &meas_cnt, sizeof(meas_cnt));
        memcpy(&temp_msg_buff[3], &recv_pd_data_buff[CH1_IDX], 2);
        memcpy(&temp_msg_buff[5], &temperature_data_buff[CH1_IDX], 2);
        memcpy(&temp_msg_buff[7], &recv_pd_data_buff[CH2_IDX], 2);
        memcpy(&temp_msg_buff[9], &temperature_data_buff[CH2_IDX], 2);
        memcpy(&temp_msg_buff[11], &recv_pd_data_buff[CH3_IDX], 2);
        memcpy(&temp_msg_buff[13], &temperature_data_buff[CH3_IDX], 2);

        return _mmi_send(MMI_CMD1_MEAS_REQ_RESP, MMI_CMD2_MEAS_REQ_DATA_W_RESP, MMI_CMD3_MEAS_REQ_SINGLE_MODE, MMI_CMD3_MEAS_REQ_SINGLE_RESP_CH_ALL_DATA_LEN, &temp_msg_buff[0]);
    }
}

HAL_StatusTypeDef Task_MMI_SendMonitorPdResult(MeasSetChVal_t ch_cfg) {
    int16_t monitor_pd_data[3];
    uint8_t data_buff[MMI_CMD3_MEAS_REQ_ADC_MAX_DATA_LEN];

    SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_MONITOR_PD_ADC, (uint16_t* )&monitor_pd_data[0]));

    if (ch_cfg == MEAS_SET_CH_MAX) {
        data_buff[0] = (monitor_pd_data[CH1_IDX] >> 8) && 0xFF;
        data_buff[1] = monitor_pd_data[CH1_IDX] && 0xFF;
        data_buff[0] = (monitor_pd_data[CH2_IDX] >> 8) && 0xFF;
        data_buff[1] = monitor_pd_data[CH2_IDX] && 0xFF;
        data_buff[0] = (monitor_pd_data[CH3_IDX] >> 8) && 0xFF;
        data_buff[1] = monitor_pd_data[CH3_IDX] && 0xFF;
        return _mmi_send(MMI_CMD1_MEAS_SET_RESP, MMI_CMD2_MEAS_SET_RESP_LED_LEVEL, ch_cfg, MMI_CMD3_MEAS_REQ_ADC_MAX_DATA_LEN, &data_buff[0]);
    }
    else {
        data_buff[0] = (monitor_pd_data[ch_cfg - 1] >> 8) && 0xFF;
        data_buff[1] = monitor_pd_data[ch_cfg - 1] && 0xFF;
        return _mmi_send(MMI_CMD1_MEAS_SET_RESP, MMI_CMD2_MEAS_SET_RESP_LED_LEVEL, ch_cfg, MMI_CMD3_MEAS_REQ_ADC_MIN_DATA_LEN, &data_buff[0]);
    }
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
    orig_arr[MMI_PROTOCOL_IDX_LEN + 1] = data_len & 0xFF;
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
    uint8_t bitsttf_line[MMI_PROTOCOL_RX_MSG_LEN_MAX * 2] = {
        0, };
    uint8_t arr_idx = 0;
    uint8_t chg_cnt = 0;

    for (arr_idx = 0; arr_idx < arr_len; arr_idx++) {
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

                    default:
                        SYS_LOG_WARN("Unexpected SPC code: %02X", p_org_line_arr[arr_idx + chg_cnt]);
                        break;
                }
                break;

            case MMI_PROTOCOL_ETX:
                bitsttf_line[arr_idx] = p_org_line_arr[arr_idx + chg_cnt];
                if (_protocol_chksum_check(bitsttf_line, arr_idx)) {
                    _process_command(bitsttf_line, arr_idx);
                    break;
                }
                else {
                    SYS_LOG_ERR("Checksum failed");
                    break;
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

static bool _protocol_chksum_check(uint8_t *arr, uint8_t cnt) {
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
            SYS_LOG_INFO("[Command 1-1: 0x%02X] Get device information", cmd1);
            _process_get_device_info(cmd2, cmd3);
            break;

        case MMI_CMD1_MEAS_SET:
            SYS_LOG_INFO("[Command 1-2: 0x%02X] Set measure settings", cmd1);
            _process_set_meas(cmd2, cmd3, p_data, data_len);
            break;

        case MMI_CMD1_MEAS_REQ:
            SYS_LOG_INFO("[Command 1-3: 0x%02X] Request measure", cmd1);
            _process_req_meas(cmd2, cmd3, p_data, data_len);
            break;

        case MMI_CMD1_REQ_DEVICE_STATUS:
            SYS_LOG_INFO("[Command 1-4: 0x%02X] Request device status", cmd1);
            _process_get_device_status(cmd2, cmd3);
            break;

        case MMI_CMD1_CTRL_DEVICE:
            SYS_LOG_INFO("[Command 1-5: 0x%02X] control device", cmd1);
            _process_ctrl_device(cmd2, cmd3, p_data, data_len);
            break;

        default:
            SYS_LOG_ERR("Invalid cmd1 value: 0X%02X", cmd1);
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
            device_info_data_val[0] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 24) & 0xFF;
            device_info_data_val[1] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 16) & 0xFF;
            device_info_data_val[2] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 8) & 0xFF;
            device_info_data_val[3] = MMI_PROTOCOL_WHO_AM_I_VAL & 0xFF;
            device_info_data_val[4] = SYS_FW_MAJOR_VER;
            device_info_data_val[5] = SYS_FW_MINOR_VER;
            device_info_data_val[6] = (SYS_FW_PATCH_VER >> 8) & 0xFF;
            device_info_data_val[7] = SYS_FW_PATCH_VER & 0xFF;
            break;

        case MMI_CMD2_INFO_WHO_AM_I_W_RESP:
            SYS_VERIFY_TRUE(MMI_CMD3_INFO_WHO_AM_I == cmd3);
            device_info_data_len = MMI_CMD3_INFO_WHO_AM_I_DATA_LEN;
            device_info_data_val[0] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 24) & 0xFF;
            device_info_data_val[1] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 16) & 0xFF;
            device_info_data_val[2] = (MMI_PROTOCOL_WHO_AM_I_VAL >> 8) & 0xFF;
            device_info_data_val[3] = MMI_PROTOCOL_WHO_AM_I_VAL & 0xFF;
            break;

        case MMI_CMD2_INFO_DEVICE_I_W_RESP:
            SYS_VERIFY_TRUE(MMI_CMD3_INFO_DEVICE == cmd3);
            device_info_data_len = MMI_CMD3_INFO_DEVICE_DATA_LEN;
            device_info_data_val[0] = SYS_FW_MAJOR_VER;
            device_info_data_val[1] = SYS_FW_MINOR_VER;
            device_info_data_val[2] = (SYS_FW_PATCH_VER >> 8) & 0xFF;
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
    MeasSetData_t cur_set_data;
    uint8_t set_data_val[MMI_CMD1_MEAS_SET_MAX_DATA_LEN];

    if (MMI_CMD2_MEAS_SET_VAL_REQ_W_RESP == cmd2) {
        SYS_VERIFY_TRUE(MMI_CMD3_MEAS_SET_VAL_REQ == cmd3);

        Task_Meas_Get_Set(&cur_set_data);
        memset(&set_data_val, 0, sizeof(set_data_val));
        memcpy(&set_data_val[0], &cur_set_data.temp_ctrl_mode[CH1_IDX], 1);
        memcpy(&set_data_val[1], &cur_set_data.temp_ctrl_mode[CH2_IDX], 1);
        memcpy(&set_data_val[2], &cur_set_data.temp_ctrl_mode[CH3_IDX], 1);
        memcpy(&set_data_val[3], &cur_set_data.led_on_time[CH1_IDX], 2);
        memcpy(&set_data_val[5], &cur_set_data.led_on_time[CH2_IDX], 2);
        memcpy(&set_data_val[7], &cur_set_data.led_on_time[CH3_IDX], 2);
        memcpy(&set_data_val[9], &cur_set_data.led_on_level[CH1_IDX], 2);
        memcpy(&set_data_val[11], &cur_set_data.led_on_level[CH2_IDX], 2);
        memcpy(&set_data_val[13], &cur_set_data.led_on_level[CH3_IDX], 2);
        memcpy(&set_data_val[15], &cur_set_data.temperature_offset[CH1_IDX], 2);
        memcpy(&set_data_val[17], &cur_set_data.temperature_offset[CH2_IDX], 2);
        memcpy(&set_data_val[19], &cur_set_data.temperature_offset[CH3_IDX], 2);
        /* 20 - 26: Reserved */
        memcpy(&set_data_val[27], &cur_set_data.stable_temperature, 2);

        SYS_VERIFY_SUCCESS(_mmi_send(MMI_CMD1_MEAS_SET_RESP, cmd2, cmd3, MMI_CMD3_MEAS_SET_VAL_REQ_RESP_DATA_LEN, (uint8_t *)&set_data_val));

        return HAL_OK;
    }
    else if (MMI_CMD2_MEAS_SET_INITIALIZE == cmd2) {
        SYS_VERIFY_TRUE(MMI_CMD3_MEAS_SET_INITIALIZE == cmd3);

        Task_Meas_Set_Initialize();

        return HAL_OK;
    }
    else {
        MeasSetChVal_t ch_cfg = (MeasSetChVal_t) cmd3; /* cmd3: ch select */

        memcpy(set_data_val, p_data, (data_len > sizeof(set_data_val) ? sizeof(set_data_val) : data_len));

        switch (cmd2) {
            case MMI_CMD2_MEAS_SET_TEMP:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_TEMP_ONOFF_DATA_LEN);

                return Task_Meas_Apply_Set(MEAS_SET_CAT_TEMP_ON_OFF, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_LED_ON_TIME:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_LED_ON_TIME_DATA_LEN);

                return Task_Meas_Apply_Set(MEAS_SET_CAT_LED_ON_TIME, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_LED_LEVEL_W_RESP:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_LED_LEVEL_DATA_LEN);

                return Task_Meas_Apply_Set(MEAS_SET_CAT_LED_ON_LEVEL, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_ADC_SAMPLE_CNT:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_ADC_SAMPLE_CNT_DATA_LEN);

                return Task_Meas_Apply_Set(MEAS_SET_CAT_ADC_SAMPLE_CNT, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_ADC_DELAY_MS:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_ADC_DELAY_DATA_LEN);

                return Task_Meas_Apply_Set(MEAS_SET_CAT_ADC_ON_DELAY, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_STABLE_TEMPERATURE:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_STABLE_TEMPERATURE_DEGREE_DATA_LEN);

                return Task_Meas_Apply_Set(MEAS_SET_CAT_STABLE_TEMPERATURE, ch_cfg, set_data_val);

            case MMI_CMD2_MEAS_SET_TEMPERATURE_OFFSET:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_SET_TEMPERATURE_OFFSET_DEGREE_DATA_LEN);

                return Task_Meas_Apply_Set(MEAS_SET_CAT_TEMPERATURE_OFFSET, ch_cfg, set_data_val);

            default:
                SYS_LOG_ERR("Invalid cmd2 value: %d", cmd2);
                return HAL_ERROR;
        }
    }
}

static HAL_StatusTypeDef _process_req_meas(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len) {
    MeasResultData_t result_data_val;
    MeasSetChVal_t set_ch;

    if (MMI_CMD2_MEAS_REQ_DATA_W_RESP == cmd2) {
        switch (cmd3) {
            case MMI_CMD3_MEAS_REQ_CONTINUOS_MODE:
                Task_Meas_Req_ContinuosMode();
                return HAL_OK;

            case MMI_CMD3_MEAS_REQ_SINGLE_MODE:
                SYS_VERIFY_TRUE(data_len == MMI_CMD3_MEAS_REQ_SINGLE_DATA_LEN);
                MeasSetChVal_t ch_cfg = p_data[0];
                uint16_t ch_cnt = (p_data[1] << 8) | p_data[2];

                if (ch_cfg < MEAS_SET_CH_ALL && ch_cfg >= MEAS_SET_CH_1) {
                    set_ch = ch_cfg;
                }
                else {
                    set_ch = MEAS_SET_CH_ALL;
                }
                Task_Meas_Req_SingleMode(set_ch, ch_cnt);
                return HAL_OK;

            default:
                SYS_LOG_ERR("Invalid cmd2 value: %d", cmd2);
                return HAL_ERROR;
        }
    }
    else {
        MeasSetChVal_t ch_cfg = (MeasSetChVal_t) cmd3; /* cmd3: ch select */

        uint8_t result_data_len = (MEAS_SET_CH_ALL == ch_cfg) ? MMI_CMD3_MEAS_REQ_ADC_MAX_DATA_LEN : MMI_CMD3_MEAS_REQ_ADC_MIN_DATA_LEN;

        switch (cmd2) {
            case MMI_CMD2_MEAS_REQ_TEMPERATURE_W_RESP:
                SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_TEMPERATURE, (uint16_t* )&result_data_val.temperature_data[0]));
                return _mmi_send(MMI_CMD1_MEAS_REQ_RESP, cmd2, cmd3, result_data_len, (uint8_t*) &result_data_val.temperature_data[0]);

            case MMI_CMD2_MEAS_REQ_RESP_ADC_W_RESP:
                SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_RECV_PD_ADC, (uint16_t* )&result_data_val.recv_pd_data[0]));
                return _mmi_send(MMI_CMD1_MEAS_REQ_RESP, cmd2, cmd3, result_data_len, (uint8_t*) &result_data_val.recv_pd_data[0]);

            case MMI_CMD2_MEAS_REQ_MONITOR_ADC_W_RESP:
                SYS_VERIFY_SUCCESS(Task_Meas_Get_Result(MEAS_RESULT_CAT_MONITOR_PD_ADC, (uint16_t* )&result_data_val.monitor_pd_data[0]));
                return _mmi_send(MMI_CMD1_MEAS_REQ_RESP, cmd2, cmd3, result_data_len, (uint8_t*) &result_data_val.monitor_pd_data[0]);

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
    
    return _mmi_send(MMI_CMD1_REQ_DEVICE_STATUS_RESP, cmd2, cmd3, MMI_CMD3_REQ_DEVICE_STATUS_DATA_LEN, &status_data_buff.msg[0]);
}

static HAL_StatusTypeDef _process_ctrl_device(uint8_t cmd2, uint8_t cmd3, uint8_t *p_data, uint8_t data_len) {
    MeasSetChVal_t ch_cfg = (MeasSetChVal_t) cmd3; /* cmd3: ch select */
    uint8_t set_data_val[MMI_CMD1_CTRL_DEVICE_MAX_DATA_LEN];

    SYS_VERIFY_TRUE(data_len <= MMI_CMD1_CTRL_DEVICE_MAX_DATA_LEN);
    memcpy(set_data_val, p_data, data_len);

    switch (cmd2) {
        case MMI_CMD2_CTRL_DEVICE_LED:
            SYS_VERIFY_TRUE(data_len == MMI_CMD3_CTRL_DEVICE_LED_DATA_LEN);

            Task_Meas_Ctrl_Led(ch_cfg, set_data_val[0]);
            break;

        case MMI_CMD2_CTRL_DATA_SEND_MODE:
            SYS_VERIFY_TRUE(data_len == MMI_CMD3_CTRL_DATA_SEND_MODE_DATA_LEN);

            Task_Meas_Ctrl_OpMode(set_data_val[0]);
            break;

        case MMI_CMD2_CTRL_ADC_INPUT_MODE:
            SYS_VERIFY_TRUE(data_len == MMI_CMD3_CTRL_ADC_INPUT_MODE_DATA_LEN);

            Task_Meas_Ctrl_AdcInputMode(set_data_val[0]);
            break;

        default:
            return HAL_ERROR;
            break;
    }

    /* FRAM Write */
    SYS_VERIFY_SUCCESS(Hal_Fram_Write(FRAM_DEV_CTRL_LED_STATUS_ADDR_CH1 + (ch_cfg-1), FRAM_DEV_CTRL_LED_STATUS_DATA_LEN, p_data));
    
    return HAL_OK;
}

