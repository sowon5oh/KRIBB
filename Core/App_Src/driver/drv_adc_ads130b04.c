/**
 ******************************************************************************
 * @file           : drv_adc_ads130b04.c
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
#include <math.h>
#include "main.h"
#include "hal_drv_pd.h"
#include "drv_adc_ads130b04.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    ADS130B04_STATE_STANDBY = 0,
    ADS130B04_STATE_CONTINOUS, /* ADC data are generated constantly at the rate of fDATA = fMOD / OSR. New data are indicated by a DRDY falling edge at this rate */
    ADS130B04_STATE_GLOBAL_CHOP, /* enabled by setting the GC_EN bit in the GLOBAL_CHOP_CFG register */
} ads130b04StateMode_t;

typedef enum {
    ADS130B04_CLOCK_INTERNAL_OSC = 0,
    ADS130B04_CLOCK_EXTERNAL_CLOCK,
} ads130b04ClockSel_t;

typedef enum {
    ADS130B04_CRC_DISABLE = 0,
    ADS130B04_CRC_ENABLE,
} ads130b04Crc_t;

/* SPS: Samples per seconds */
typedef enum {
    ADS130B04_OSR_MODE_128 = 0, /* 32 	 ksps Default */
    ADS130B04_OSR_MODE_256, /* 16 	 ksps */
    ADS130B04_OSR_MODE_512, /*  8 	 ksps */
    ADS130B04_OSR_MODE_1024, /*  4	 ksps */
    ADS130B04_OSR_MODE_2048, /*  2 	 ksps */
    ADS130B04_OSR_MODE_4096, /*  1 	 ksps */
    ADS130B04_OSR_MODE_8192, /*  0.5  ksps */
    ADS130B04_OSR_MODE_16384, /*  0.25 ksps */
} ads130b04OsrMode_t;

typedef enum {
    ADS130B04_PWR_MODE_VERY_LOW_POWER = 0,
    ADS130B04_PWR_MODE_LOW_POWER,
    ADS130B04_PWR_MODE_HIGH_RESOLUTION,
} ads130b04PwrMode_t;

typedef enum {
    ADS130B04_SYNC_MODE_OFF = 0, /* Default */
    ADS130B04_SYNC_MODE_ON,
} ads130b04SyncMode_t;

typedef enum {
    ADS130B04_WORD_SIZE_16_BIT = 0,
    ADS130B04_WORD_SIZE_24_BIT, /* Default */
    ADS130B04_WORD_SIZE_32_BIT,
} ads130b04SpiWordSize_t;

typedef struct {
    ads130b04Crc_t crc_enable;
    ads130b04SpiWordSize_t size_id;
    uint8_t byte_num;
} ads130b04SpiWordCfg_t;

typedef enum {
    ADS130B04_CH_GAIN_MODE_1 = 0, /* FSR: +/- 1.2     V */
    ADS130B04_CH_GAIN_MODE_2, /* FSR: +/- 600    mV */
    ADS130B04_CH_GAIN_MODE_4, /* FSR: +/- 300    mV */
    ADS130B04_CH_GAIN_MODE_8, /* FSR: +/- 150    mV */
    ADS130B04_CH_GAIN_MODE_16, /* FSR: +/-  75    mV */
    ADS130B04_CH_GAIN_MODE_32, /* FSR: +/-  37.5  mV */
    ADS130B04_CH_GAIN_MODE_64, /* FSR: +/-  18.75 mV */
    ADS130B04_CH_GAIN_MODE_128, /* FSR: +/-  9.375 mV */
} ads130b04ChGainMode_t;

typedef struct {
    bool enable;
    Ads130b04InputMode_t input_mode;
} ads130b04ChCfg_t;

typedef enum {
    ADS130B04_CMD_ID_NULL = 0,
    ADS130B04_CMD_ID_RESET,
    ADS130B04_CMD_ID_STANDBY,
    ADS130B04_CMD_ID_WAKEUP,
    ADS130B04_CMD_ID_LOCK,
    ADS130B04_CMD_ID_UNLOCK,
} ads130b04CmdId_t;

typedef struct {
    SPI_HandleTypeDef *spi_handle;
    HalPdMeasRespCb_t cb_fn;
    Ads130b04Ch3MuxCh_t cur_mux_ch;
} ads130b04Context_t;

typedef struct {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3_0; /* Mux CH 1 */
    int16_t ch3_1; /* Mux CH 2 */
    int16_t ch3_2; /* Mux CH 3 */
} ads130b04Result_t;

/* Private define ------------------------------------------------------------*/
#define ADS130B04_SPI_Select()                HAL_GPIO_WritePin(ADC_CS__GPIO_Port, ADC_CS__Pin, GPIO_PIN_RESET)
#define ADS130B04_SPI_Deselect()              HAL_GPIO_WritePin(ADC_CS__GPIO_Port, ADC_CS__Pin, GPIO_PIN_SET)

#define ADS130B04_CMD_NULL                    0x0000  /* word: 0000 0000 0000 0000 */
#define ADS130B04_CMD_RESET                   0x0011  /* word: 0000 0000 0001 0001 */
#define ADS130B04_CMD_STANDBY                 0x0022  /* word: 0000 0000 0010 0010, Places the device in a low-power standby mode */
#define ADS130B04_CMD_WAKEUP                  0x0033  /* word: 0000 0000 0011 0011, Returns the device to conversion mode from standby mode */
#define ADS130B04_CMD_LOCK                    0x0555  /* word: 0000 0101 0101 0101 */
#define ADS130B04_CMD_UNLOCK                  0x0655  /* word: 0000 0110 0101 0101 */

#define ADS130B04_CMD_RESP_RESET              0xFF54  /* word: 1111 1111 0101 0100 */
#define ADS130B04_CMD_RESP_STANDBY            ADS130B04_CMD_STANDBY
#define ADS130B04_CMD_RESP_WAKEUP             ADS130B04_CMD_WAKEUP
#define ADS130B04_CMD_RESP_LOCK               ADS130B04_CMD_LOCK
#define ADS130B04_CMD_RESP_UNLOCK             ADS130B04_CMD_UNLOCK

#define ADS130B04_CMD_RREG_HEAD               0xA000  /* Used to read the device registers */
#define ADS130B04_CMD_WREG_HEAD               0x6000
#define ADS130B04_CMD_W_R_REG_ADDR_BOFF       7
#define ADS130B04_CMD_W_R_REG_ADDR_BCNT       6
#define ADS130B04_CMD_W_R_DATA_NUM_BOFF       0
#define ADS130B04_CMD_W_R_DATA_NUM_BCNT       7
#define ADS130B04_CMD_RREG_MAX_LEN            10

#define ADS130B04_ID                          0x54 /* 0b0101 0100 XXXX XXXX */
/* DEVICE SETTINGS AND STATUS INDICATORS (Read-Only Registers) */
#define ADS130B04_REG_ADDR_ID                 0x00
#define ADS130B04_ID_BOFF                     8
#define ADS130B04_ID_BCNT                     8
#define ADS130B04_REG_ADDR_STATUS             0x01
#define ADS130B04_CH0_DRDY_BOFF               0
#define ADS130B04_CH0_DRDY_BCNT               1
#define ADS130B04_CH1_DRDY_BOFF               1
#define ADS130B04_CH1_DRDY_BCNT               1
#define ADS130B04_CH2_DRDY_BOFF               2
#define ADS130B04_CH2_DRDY_BCNT               1
#define ADS130B04_CH3_DRDY_BOFF               3
#define ADS130B04_CH3_DRDY_BCNT               1
#define ADS130B04_WLENGTH_BCNT                2
#define ADS130B04_WLENGTH_BOFF                8
/* GLOBAL SETTINGS ACROSS CHANNELS */
#define ADS130B04_REG_ADDR_MODE               0x02
#define ADS130B04_REG_ADDR_CLOCK              0x03
#define ADS130B04_PWR_MODE_BOFF               0
#define ADS130B04_PWR_MODE_BCNT               2
#define ADS130B04_OSR_MODE_BOFF               2
#define ADS130B04_OSR_MODE_BCNT               3
#define ADS130B04_CLK_SEL_BOFF                7
#define ADS130B04_CLK_SEL_BCNT                1
#define ADS130B04_CH0_EN_BOFF                 8
#define ADS130B04_CH0_EN_BCNT                 1
#define ADS130B04_CH1_EN_BOFF                 9
#define ADS130B04_CH1_EN_BCNT                 1
#define ADS130B04_CH2_EN_BOFF                 10
#define ADS130B04_CH2_EN_BCNT                 1
#define ADS130B04_CH3_EN_BOFF                 11
#define ADS130B04_CH3_EN_BCNT                 1
#define ADS130B04_REG_ADDR_GAIN               0x04
#define ADS130B04_GAIN_CFG_CH0_BOFF           0
#define ADS130B04_GAIN_CFG_CH0_BCNT           3
#define ADS130B04_GAIN_CFG_CH1_BOFF           4
#define ADS130B04_GAIN_CFG_CH1_BCNT           3
#define ADS130B04_GAIN_CFG_CH2_BOFF           8
#define ADS130B04_GAIN_CFG_CH2_BCNT           3
#define ADS130B04_GAIN_CFG_CH3_BOFF           12
#define ADS130B04_GAIN_CFG_CH3_BCNT           3
#define ADS130B04_REG_ADDR_GLOBAL_CHOP_CFG    0x06
/* CHANNEL-SPECIFIC SETTINGS */
#define ADS130B04_REG_ADDR_CH0_CFG            0x09    // 9
#define ADS130B04_REG_ADDR_CH1_CFG            0x0E    // 14
#define ADS130B04_REG_ADDR_CH2_CFG            0x13    // 19
#define ADS130B04_REG_ADDR_CH3_CFG            0x18    // 24
#define DRV_ADS130B04_CH_CFG_MUX_BOFF         0
#define DRV_ADS130B04_CH_CFG_MUX_BCNT         2
/* REGISTER MAP CRC REGISTER (Read-Only Register) */
#define ADS130B04_REG_ADDR_REG_MAP_CRC        0x3E    // 62

#define ADS130B04_SPI_WORD_BYTE_NUM_16BIT     2
#define ADS130B04_SPI_WORD_BYTE_NUM_24BIT     3
#define ADS130B04_SPI_WORD_BYTE_NUM_32BIT     4

#define ADS130B04_SPI_WORD_MIN_BUFF_SIZE      (ADS130B04_SPI_WORD_BYTE_NUM_32BIT * 2)  /* Command, Crc */
#define ADS130B04_SPI_WORD_MAX_BUFF_SIZE      (ADS130B04_SPI_WORD_MIN_BUFF_SIZE * 6)   /* Command, Data0, Data1, Data2, Data3, Crc */

#define ADS130B04_SEND_CMD_TX_LEN  2 /* [Command] + [CRC] */
#define ADS130B04_SEND_CMD_IDX_CMD 0
#define ADS130B04_SEND_CMD_IDX_CRC 1
#define ADS130B04_READ_DATA_RX_LEN 6 /* [Status] [CH0] [CH1] [CH2] [CH3] [CRC] */

#define ADS130B04_WRITE_REG_SINGLE_TX_LEN   3 /* [Command] [Write Data] [CRC] */
#define ADS130B04_WRITE_REG_SINGLE_IDX_CMD  0
#define ADS130B04_WRITE_REG_SINGLE_IDX_DATA 1
#define ADS130B04_WRITE_REG_SINGLE_IDX_CRC  2
#define ADS130B04_WRITE_REG_RESP_RX_LEN     1 /* [Command Resp] */

#define ADS130B04_OSR_MODE_DEFAULT   ADS130B04_OSR_MODE_256
#define ADS130B04_CLOCK_DEFAULT      ADS130B04_CLOCK_INTERNAL_OSC
#define ADS130B04_PWR_MODE_DEFAULT   ADS130B04_PWR_MODE_HIGH_RESOLUTION

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef _meas_enable(bool enable);
static HAL_StatusTypeDef _send_cmd(ads130b04CmdId_t cmd);
static HAL_StatusTypeDef _fetch_adc(void);
static HAL_StatusTypeDef _set_clock_cfg(void);
static HAL_StatusTypeDef _set_gain_cfg(void);
static HAL_StatusTypeDef _set_ch_mux_cfg(void);

static HAL_StatusTypeDef _register_sigle_write(uint16_t reg, uint16_t data);
static HAL_StatusTypeDef _register_read(uint16_t reg, uint8_t len, uint16_t *p_data);

static HAL_StatusTypeDef _comm_tx_rx(uint16_t *p_tx_data, uint8_t tx_len, int16_t *p_rx_data, uint8_t rx_len);
static HAL_StatusTypeDef _comm_tx(uint16_t *p_data, uint8_t len);
static HAL_StatusTypeDef _comm_rx(int16_t *p_data, uint8_t len);

static void _make_word_packet(uint16_t *p_data, uint8_t len, uint8_t *p_result);
static void _parse_word_packet(uint8_t *p_data, uint8_t len, int16_t *p_result);
static uint16_t _make_crc(uint16_t *p_data);

static void _ch4_mux_enable(bool enable);
static void _ch4_mux_select(Ads130b04Ch3MuxCh_t ch);
/* Private variables ---------------------------------------------------------*/
ads130b04Context_t ads130b04_context = {
    .cur_mux_ch = DRV_ADS130B04_MUX_CH_0,
};
/* ADS130B04 Settings */
static ads130b04StateMode_t ads130b04_state_mode;
static bool ads130b04_lock;
static bool ads130b04_global_chop;
static ads130b04SyncMode_t ads130b04_sync_mode;
static ads130b04SpiWordCfg_t ads130b04_word_size;
/* ADS130B04 Clock Config */
static ads130b04ClockSel_t ads130b04_clock_sel;
static ads130b04OsrMode_t ads130b04_osr_mode;
static ads130b04PwrMode_t ads130b04_pwr_mode;
static ads130b04ChGainMode_t ads130b04_gain_mode;
static double ads_130b04_lsb;
static ads130b04ChCfg_t ads130b04_ch_cfg[DRV_ADS130B04_CH_NUM];
static ads130b04Result_t ads130b04_result;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_ADS130B04_Init(SPI_HandleTypeDef *p_hdl, HalPdMeasRespCb_t cb_fn) {
    uint16_t read_data = 0;
    
    /* attach spi handle */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    ads130b04_context.spi_handle = p_hdl;
    
    /* attach callback function */
    if (cb_fn != NULL) {
        ads130b04_context.cb_fn = cb_fn;
    }
    
    /* Pin Reset */
    HAL_GPIO_WritePin(ADC_RST__GPIO_Port, ADC_RST__Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(ADC_RST__GPIO_Port, ADC_RST__Pin, GPIO_PIN_SET);
    
    /* General Config */
    ads130b04_lock = false;
    ads130b04_global_chop = false;
    ads130b04_sync_mode = ADS130B04_SYNC_MODE_OFF;
    
    ads130b04_word_size.crc_enable = ADS130B04_CRC_DISABLE;
    ads130b04_word_size.size_id = ADS130B04_WORD_SIZE_24_BIT;
    if (ADS130B04_WORD_SIZE_24_BIT == ads130b04_word_size.size_id) {
        ads130b04_word_size.byte_num = ADS130B04_SPI_WORD_BYTE_NUM_24BIT;
    }
    else if (ADS130B04_WORD_SIZE_16_BIT == ads130b04_word_size.size_id) {
        ads130b04_word_size.byte_num = ADS130B04_SPI_WORD_BYTE_NUM_16BIT;
    }
    else if (ADS130B04_WORD_SIZE_32_BIT == ads130b04_word_size.size_id) {
        ads130b04_word_size.byte_num = ADS130B04_SPI_WORD_BYTE_NUM_32BIT;
    }
    
    /* Clock Config */
    ads130b04_clock_sel = ADS130B04_CLOCK_DEFAULT;
    ads130b04_osr_mode = ADS130B04_OSR_MODE_DEFAULT;
    ads130b04_pwr_mode = ADS130B04_PWR_MODE_DEFAULT;
    /* Ch0 Config */
    ads130b04_ch_cfg[DRV_ADS130B04_CH_0].enable = true;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_0].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    /* Ch1 Config */
    ads130b04_ch_cfg[DRV_ADS130B04_CH_1].enable = true;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_1].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    /* Ch2 Config */
    ads130b04_ch_cfg[DRV_ADS130B04_CH_2].enable = true;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_2].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    /* Ch3 Config */
    ads130b04_ch_cfg[DRV_ADS130B04_CH_3].enable = true;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_3].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    
    /* Gain Config */
    ads130b04_gain_mode = ADS130B04_CH_GAIN_MODE_1;
    double fsr = (1.2 / ads130b04_gain_mode); /* Full Scale Range: (+/-1.2V)/gain */
    ads_130b04_lsb = fsr / pow(2, 15); /* LSB: 1 LSB = (2.4 / Gain) / 2^16 = +FSR / 2^15 */
    /* if Gain is 1,    1 LSB = 36.62109 uV */
    /* if Gain is 128,  1 LSB =  0.00028 uV */

    /* Read ID */
    SYS_VERIFY_SUCCESS(_register_read(ADS130B04_REG_ADDR_ID, 1, &read_data));

    //0x5404 - 21508
    SYS_LOG_DEBUG("[ADC ID: %04X]", read_data);
    uint8_t dev_id = BF_GET(read_data, ADS130B04_ID_BCNT, ADS130B04_ID_BOFF);
    SYS_VERIFY_TRUE(ADS130B04_ID == dev_id);
    
    SYS_VERIFY_SUCCESS(_register_read(ADS130B04_REG_ADDR_STATUS, 1, &read_data));
    //0x0500 - 1280
    SYS_LOG_DEBUG("[ADC STATUS: %04X]", read_data); //
    SYS_LOG_DEBUG("- CH DRDY STATUS: %04X", BF_GET(read_data, 4, ADS130B04_CH0_DRDY_BOFF)); // CH DRDY
    SYS_LOG_DEBUG("- WLENGTH       : %02x", BF_GET(read_data,ADS130B04_WLENGTH_BCNT, ADS130B04_WLENGTH_BOFF )); //WLENGTH

    SYS_VERIFY_SUCCESS(_register_read(ADS130B04_REG_ADDR_MODE, 1, &read_data));
    //0x0510 - 1296
    SYS_LOG_DEBUG("[ADC MODE: %04X]", read_data);

//    /* Reset */
//    SYS_VERIFY_SUCCESS(_send_cmd(ADS130B04_CMD_ID_RESET));
    
    /* Standby */
    SYS_VERIFY_SUCCESS(_send_cmd(ADS130B04_CMD_ID_STANDBY));
    
    /* Apply Config */
    _set_clock_cfg();
    SYS_VERIFY_SUCCESS(_register_read(ADS130B04_REG_ADDR_CLOCK, 1, &read_data));
    /* Check Config result */
    SYS_LOG_DEBUG("[ADC CLOCK: %04X]", read_data); //0x0F8E - 3970
    SYS_LOG_DEBUG("- POWER MODE: %04X", BF_GET(read_data, ADS130B04_PWR_MODE_BCNT, ADS130B04_PWR_MODE_BOFF)); //Power Mode
    SYS_LOG_DEBUG("- OSR       : %04X", BF_GET(read_data, ADS130B04_OSR_MODE_BCNT, ADS130B04_OSR_MODE_BOFF)); //OSR
    SYS_LOG_DEBUG("- CH ENBLE  : %04X", BF_GET(read_data, 4, ADS130B04_CH0_EN_BOFF)); //CH Enable
    _set_gain_cfg();
    _set_ch_mux_cfg();
    
    /* Mux config */
    _ch4_mux_enable(false);
    _ch4_mux_select(DRV_ADS130B04_MUX_CH_0);
    
    return HAL_OK;
}

HAL_StatusTypeDef DRV_ADS130B04_Start(void) {
    /* Wake Up */
    _send_cmd(ADS130B04_CMD_ID_WAKEUP);
    HAL_Delay(100);
    _meas_enable(true);
    _ch4_mux_enable(true);
    
    return HAL_OK;
}

HAL_StatusTypeDef DRV_ADS130B04_Stop(void) {
    /* Standby */
    SYS_VERIFY_SUCCESS(_meas_enable(false));
    SYS_VERIFY_SUCCESS(_send_cmd(ADS130B04_CMD_ID_STANDBY));
    _ch4_mux_enable(false);
    
    return HAL_OK;
}

HAL_StatusTypeDef DRV_ADS130B04_SetMuxCh(Ads130b04Ch3MuxCh_t ch) {
    SYS_VERIFY_TRUE(ch < DRV_ADS130B04_MUX_CH_NUM);

    _ch4_mux_enable(false);
    _ch4_mux_select(ch);
    _ch4_mux_enable(true);

    return HAL_OK;
}

HAL_StatusTypeDef DRV_ADS130B04_SetInput(Ads130b04InputMode_t input_mode) {
    SYS_VERIFY_TRUE(input_mode < DRV_ADS130B04_CH_INPUT_MODE_TEST_NUM);

    SYS_VERIFY_SUCCESS(_send_cmd(ADS130B04_CMD_ID_STANDBY));
    ads130b04_ch_cfg[DRV_ADS130B04_CH_0].input_mode = input_mode;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_1].input_mode = input_mode;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_2].input_mode = input_mode;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_3].input_mode = input_mode;
    SYS_VERIFY_SUCCESS(_send_cmd(ADS130B04_CMD_ID_WAKEUP));

    /* Ch MUX Config */
    return _set_ch_mux_cfg();
}

HAL_StatusTypeDef DRV_ADS130B04_GetData(Ads130b04ChSel_t ch, int16_t *p_data) {
    SYS_VERIFY_TRUE(ch < DRV_ADS130B04_CH_MAX);
    SYS_VERIFY_PARAM_NOT_NULL(p_data);

    if (HAL_OK != _fetch_adc()) {
        SYS_LOG_ERR("Update ADC Value Failed");
        return HAL_ERROR;
    }

    switch (ch) {
        case DRV_ADS130B04_CH_0:
            *p_data = ads130b04_result.ch0;
            break;
            
        case DRV_ADS130B04_CH_1:
            *p_data = ads130b04_result.ch1;
            break;
            
        case DRV_ADS130B04_CH_2:
            *p_data = ads130b04_result.ch2;
            break;
            
        case DRV_ADS130B04_CH_3_0:
            *p_data = ads130b04_result.ch3_0;
            break;

        case DRV_ADS130B04_CH_3_1:
            *p_data = ads130b04_result.ch3_1;
            break;

        case DRV_ADS130B04_CH_3_2:
            *p_data = ads130b04_result.ch3_2;
            break;
            
        default:
            return HAL_ERROR;
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef DRV_ADS130B04_UpdateData(void) {
    for (uint8_t retry = 0; retry < 5; retry++) {
        if (HAL_OK == _fetch_adc()) {
            SYS_LOG_INFO("Update ADC Value Success");
            return HAL_OK;
        }
    }

    SYS_LOG_ERR("Update ADC Value Failed");
    return HAL_ERROR;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ADC_DRDY__Pin) {
        /* Add If nedded */
    }
}

/* Private user code ---------------------------------------------------------*/
static HAL_StatusTypeDef _meas_enable(bool enable) {
    /* Disable ADC Ch */
    ads130b04_ch_cfg[DRV_ADS130B04_CH_0].enable = enable;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_1].enable = enable;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_2].enable = enable;
    ads130b04_ch_cfg[DRV_ADS130B04_CH_3].enable = enable;
    
    SYS_VERIFY_SUCCESS(_set_clock_cfg());
    
    return HAL_OK;
}

static HAL_StatusTypeDef _send_cmd(ads130b04CmdId_t cmd) {
    uint16_t tx_buff[2];
    int16_t rx_buff[2];
    uint16_t cmd_send = 0;
    uint16_t cmd_resp = 0;
    
    switch (cmd) {
        case ADS130B04_CMD_ID_RESET:
            cmd_send = ADS130B04_CMD_RESET;
            cmd_resp = ADS130B04_CMD_RESP_RESET;
            SYS_LOG_DEBUG("ADC Send Command Reset: %04X", cmd_send);
            break;
            
        case ADS130B04_CMD_ID_STANDBY:
            cmd_send = ADS130B04_CMD_STANDBY;
            cmd_resp = ADS130B04_CMD_RESP_STANDBY;
            ads130b04_state_mode = ADS130B04_STATE_STANDBY;
            SYS_LOG_DEBUG("ADC Send Command Standby: %04X", cmd_send);
            break;
            
        case ADS130B04_CMD_ID_WAKEUP:
            cmd_send = ADS130B04_CMD_WAKEUP;
            cmd_resp = ADS130B04_CMD_RESP_WAKEUP;
            if (ads130b04_global_chop == false) {
                ads130b04_state_mode = ADS130B04_STATE_CONTINOUS;
            }
            else {
                ads130b04_state_mode = ADS130B04_STATE_GLOBAL_CHOP;
            }
            SYS_LOG_DEBUG("ADC Send Command Wakeup: %04X", cmd_send);
            break;
            
        case ADS130B04_CMD_ID_LOCK:
            cmd_send = ADS130B04_CMD_LOCK;
            cmd_resp = ADS130B04_CMD_RESP_LOCK;
            ads130b04_lock = true;
            SYS_LOG_DEBUG("ADC Send Command Lock: %04X", cmd_send);
            break;
            
        case ADS130B04_CMD_ID_UNLOCK:
            cmd_send = ADS130B04_CMD_UNLOCK;
            cmd_resp = ADS130B04_CMD_RESP_UNLOCK;
            ads130b04_lock = false;
            SYS_LOG_DEBUG("ADC Send Command Unlock: %04X", cmd_send);
            break;
            
        default:
            SYS_LOG_ERR("Not supported command %04X", cmd_send);
            return HAL_ERROR;
    }
    
    /* Make CMD Array: [CMD] [CRC] */
    tx_buff[0] = cmd_send;
    if (ADS130B04_CRC_ENABLE == ads130b04_word_size.crc_enable) {
        tx_buff[ADS130B04_SEND_CMD_IDX_CRC] = _make_crc(&tx_buff[ADS130B04_SEND_CMD_IDX_CMD]);
        return HAL_ERROR;
    }
    else {
        tx_buff[ADS130B04_SEND_CMD_IDX_CRC] = 0; /* CRC */
    }
    
    if (HAL_OK == _comm_tx_rx(&tx_buff[0], 2, &rx_buff[0], 1)) {
        if (cmd_resp == rx_buff[0]) {
            SYS_LOG_DEBUG("ADC Command valid response: %04X", rx_buff[0]);
            return HAL_OK;
        }
        else {
            SYS_LOG_ERR("ADC Command wrong response: %04X", rx_buff[0]);
            return HAL_ERROR;
        }
    }
    else {
        SYS_LOG_ERR("Send command failed");
        return HAL_ERROR;
    }
}

/*
 * NULL Command Format: 0000 0000 0000 0000
 *
 * - The NULL command is a no-operation command (NOP) that:
 *   - Does not read or write any registers.
 *   - Leaves the device state unchanged.
 *
 * - Use Case:
 *   - The primary purpose of the NULL command is to allow the reading of ADC conversion data.
 *   - When the NULL command is issued, the command response contains the contents of the STATUS register.
 *
 * - Invalid Command:
 *   - If an invalid command is sent, the device responds as if the NULL command was received,
 *     outputting the STATUS register contents as the response.
 */
static HAL_StatusTypeDef _fetch_adc(void) {
    uint8_t drdy_status = 0;
    uint16_t tx_buff[ADS130B04_SEND_CMD_TX_LEN];
    int16_t rx_buff[ADS130B04_READ_DATA_RX_LEN];
    
    tx_buff[ADS130B04_SEND_CMD_IDX_CMD] = ADS130B04_CMD_NULL;
    if (ADS130B04_CRC_ENABLE == ads130b04_word_size.crc_enable) {
        tx_buff[ADS130B04_SEND_CMD_IDX_CRC] = _make_crc(&tx_buff[ADS130B04_SEND_CMD_IDX_CMD]);
    }
    else {
        tx_buff[ADS130B04_SEND_CMD_IDX_CRC] = 0; /* CRC */
    }
    
    if (HAL_OK == _comm_tx_rx(&tx_buff[0], ADS130B04_SEND_CMD_TX_LEN, &rx_buff[0], ADS130B04_READ_DATA_RX_LEN)) {
        drdy_status = BF_GET(rx_buff[0], 4, ADS130B04_CH0_DRDY_BOFF);
        if (drdy_status != 0x0f) {
            SYS_LOG_WARN("[ADC], INVALID STATUS: %0x", drdy_status);
            return HAL_ERROR;
        }
    }
    else {
        return HAL_ERROR;
    }
    
    ads130b04_result.ch0 = rx_buff[1];
    ads130b04_result.ch1 = rx_buff[2];
    ads130b04_result.ch2 = rx_buff[3];
    SYS_LOG_INFO("[ADC] %5d, %5d, %5d", ads130b04_result.ch0, ads130b04_result.ch1, ads130b04_result.ch2);
    
    switch(ads130b04_context.cur_mux_ch){
        case DRV_ADS130B04_MUX_CH_0:
            ads130b04_result.ch3_0 = rx_buff[4];
            break;

        case DRV_ADS130B04_MUX_CH_1:
            ads130b04_result.ch3_1 = rx_buff[4];
            break;

        case DRV_ADS130B04_MUX_CH_2:
            ads130b04_result.ch3_2 = rx_buff[4];
            break;

        default:
            SYS_LOG_ERR("Invalid MUX CH %d", ads130b04_context.cur_mux_ch);
            break;
    }
    SYS_LOG_INFO("[ADC MUX Cur CH: %d] %5d, %5d, %5d", ads130b04_context.cur_mux_ch, ads130b04_result.ch3_0, ads130b04_result.ch3_1, ads130b04_result.ch3_2);

    if (ads130b04_context.cb_fn != NULL) {
        ads130b04_context.cb_fn();
    }

    return HAL_OK;
}

static HAL_StatusTypeDef _set_clock_cfg(void) {
    uint16_t send_cfg = 0x00;
    
    /* Ch Enable */
    send_cfg |= BF_VAL(ads130b04_ch_cfg[DRV_ADS130B04_CH_0].enable, ADS130B04_CH0_EN_BCNT, ADS130B04_CH0_EN_BOFF);
    send_cfg |= BF_VAL(ads130b04_ch_cfg[DRV_ADS130B04_CH_1].enable, ADS130B04_CH1_EN_BCNT, ADS130B04_CH1_EN_BOFF);
    send_cfg |= BF_VAL(ads130b04_ch_cfg[DRV_ADS130B04_CH_2].enable, ADS130B04_CH2_EN_BCNT, ADS130B04_CH2_EN_BOFF);
    send_cfg |= BF_VAL(ads130b04_ch_cfg[DRV_ADS130B04_CH_3].enable, ADS130B04_CH3_EN_BCNT, ADS130B04_CH3_EN_BOFF);
    
    /* Clock Select */
    send_cfg |= BF_VAL(ads130b04_clock_sel, ADS130B04_CLK_SEL_BCNT, ADS130B04_CLK_SEL_BOFF);
    
    /* OSR Config */
    send_cfg |= BF_VAL(ads130b04_osr_mode, ADS130B04_OSR_MODE_BCNT, ADS130B04_OSR_MODE_BOFF);
    
    /* Power mode Config */
    send_cfg |= BF_VAL(ads130b04_pwr_mode, ADS130B04_PWR_MODE_BCNT, ADS130B04_PWR_MODE_BOFF);
    
    SYS_VERIFY_SUCCESS(_register_sigle_write(ADS130B04_REG_ADDR_CLOCK, send_cfg));SYS_LOG_DEBUG("ADC Set Clock Config Success: %04X", send_cfg); //0F0E - 3854

    return HAL_OK;
}

static HAL_StatusTypeDef _set_gain_cfg(void) {
    uint16_t send_cfg = 0x00;
    
    /* Clock Config */
    send_cfg |= BF_VAL(ads130b04_gain_mode, ADS130B04_GAIN_CFG_CH0_BCNT, ADS130B04_GAIN_CFG_CH0_BOFF);
    send_cfg |= BF_VAL(ads130b04_gain_mode, ADS130B04_GAIN_CFG_CH1_BCNT, ADS130B04_GAIN_CFG_CH1_BOFF);
    send_cfg |= BF_VAL(ads130b04_gain_mode, ADS130B04_GAIN_CFG_CH2_BCNT, ADS130B04_GAIN_CFG_CH2_BOFF);
    send_cfg |= BF_VAL(ads130b04_gain_mode, ADS130B04_GAIN_CFG_CH3_BCNT, ADS130B04_GAIN_CFG_CH3_BOFF);
    
    SYS_VERIFY_SUCCESS(_register_sigle_write(ADS130B04_REG_ADDR_GAIN, send_cfg));SYS_LOG_DEBUG("ADC Set Gain Config Success: %04X", send_cfg); //0000

    return HAL_OK;
}

static HAL_StatusTypeDef _set_ch_mux_cfg(void) {
    uint16_t send_cfg = 0x00;
    uint8_t input_mode[DRV_ADS130B04_CH_NUM] = {
        0, };

    input_mode[0] = ads130b04_ch_cfg[DRV_ADS130B04_CH_0].input_mode;
    input_mode[1] = ads130b04_ch_cfg[DRV_ADS130B04_CH_1].input_mode;
    input_mode[2] = ads130b04_ch_cfg[DRV_ADS130B04_CH_2].input_mode;
    input_mode[3] = ads130b04_ch_cfg[DRV_ADS130B04_CH_3].input_mode;
    
    /* Ch MUX Config */
    for (uint8_t idx = 0; idx < 4; idx++) { //5
        send_cfg = BF_VAL(input_mode[idx], DRV_ADS130B04_CH_CFG_MUX_BCNT, DRV_ADS130B04_CH_CFG_MUX_BOFF);
        SYS_VERIFY_SUCCESS(_register_sigle_write(ADS130B04_REG_ADDR_CH0_CFG + 5 * idx, send_cfg));
        SYS_LOG_INFO("ADC Set MUX CH%d Config Success: %04X", idx, send_cfg); // CH1234 success : 0000
    }
    
    return HAL_OK;
}

/* WREG Command Format: 011a aaaa annn nnnn
 *
 * - The WREG command allows writing data to one or more consecutive registers in the device.
 * - Command Word Format:
 *   - The command begins with the binary pattern 011, followed by the register address and the number of registers.
 *
 *   - Bits 15-8: 011a aaaa
 *     - a aaaa a (Bits 10-14) represents the binary address of the first register to write.
 *
 *   - Bits 7-0: annn nnnn
 *     - nnn nnnn (Bits 6-0) specifies the number of consecutive registers to write, minus one.
 *       - Example: To write to 1 register, set nnn nnnn to 0000000.
 *       - To write to 2 registers, set nnn nnnn to 0000001, and so on.
 *
 * - Data Writing:
 *   - After the command word, send the data to be written to each register in order.
 *   - Each register’s data should be sent as an individual word, aligned to the MSB.
 *   - Send the data consecutively for each register specified.
 *
 * - CRC Usage (if enabled):
 *   - Append the CRC value after the register data if CRC is enabled.
 *   - CRC errors set the CRC_ERR bit in the STATUS register, but do not block data from being written.
 *
 * - Write Restrictions:
 *   - The device ignores writes to read-only registers or invalid addresses.
 *   - Non-writable registers are counted in nnn nnnn, but remain unchanged.
 *
 * - Response:
 *   - In the following frame, the device responds with 010a aaaa ammm mmmm,
 *     where mmm mmmm indicates the number of registers actually written, minus one.
 *
 * Example:
 * - To write to registers 0x05 and 0x06, the command word would be:
 *   - 0110 1010 0000 0001 (where a aaaa a is 001010 for register 0x05, and nnn nnnn is 0000001 for 2 registers).
 */
static HAL_StatusTypeDef _register_sigle_write(uint16_t reg, uint16_t data) {
    uint16_t send_data[3] = {
        0x00, };
    int16_t resp_data = 0x00;
    uint16_t save_data = 0x00;
    uint16_t resp_chk;
    
    send_data[ADS130B04_WRITE_REG_SINGLE_IDX_CMD] = ADS130B04_CMD_WREG_HEAD;
    send_data[ADS130B04_WRITE_REG_SINGLE_IDX_CMD] |= BF_VAL(reg, ADS130B04_CMD_W_R_REG_ADDR_BCNT, ADS130B04_CMD_W_R_REG_ADDR_BOFF);
    send_data[ADS130B04_WRITE_REG_SINGLE_IDX_DATA] = data;
    if (ADS130B04_CRC_ENABLE == ads130b04_word_size.crc_enable) {
        send_data[ADS130B04_WRITE_REG_SINGLE_IDX_CRC] = _make_crc(&send_data[ADS130B04_WRITE_REG_SINGLE_IDX_CMD]);
        return HAL_ERROR;
    }
    else {
        send_data[ADS130B04_WRITE_REG_SINGLE_IDX_CRC] = 0; /* CRC */
    }
    
    SYS_VERIFY_SUCCESS(_comm_tx(&send_data[0], ADS130B04_WRITE_REG_SINGLE_TX_LEN));
    SYS_VERIFY_SUCCESS(_comm_rx(&resp_data, ADS130B04_WRITE_REG_RESP_RX_LEN));
    
    resp_chk = BF_GET(resp_data, ADS130B04_CMD_W_R_REG_ADDR_BCNT, ADS130B04_CMD_W_R_REG_ADDR_BOFF);
    if (reg != resp_chk) {
        SYS_LOG_ERR("Register Write failed, cmd don't match: %04X | %04X", reg, resp_chk);
        
        return HAL_ERROR;
    }
    
    resp_chk = BF_GET(resp_data, ADS130B04_CMD_W_R_DATA_NUM_BCNT, ADS130B04_CMD_W_R_DATA_NUM_BOFF);
    if (resp_chk != 0) {
        SYS_LOG_ERR("Register Write failed, len don't match: 0 | %d", resp_chk);
        
        return HAL_ERROR;
    }
    
    SYS_VERIFY_SUCCESS(_register_read(reg, ADS130B04_WRITE_REG_RESP_RX_LEN, &save_data));
    
    if (data != save_data) {
        SYS_LOG_ERR("Write & Read data don't match: %04X | %04X", data, save_data);
        
        return HAL_ERROR;
    }
    else {
        return HAL_OK;
    }
}

/*
 * RREG Command Format: 101a aaaa annn nnnn
 *
 * - The RREG command is used to read one or more consecutive registers from the device.
 * - Command Word Format:
 *   - The command begins with the binary pattern 101 followed by the register address and the number of registers to read.
 *
 *   - Bits 15-8: 101a aaaa
 *     - a aaaa a (Bits 10-14) specifies the binary address of the first register to read.
 *
 *   - Bits 7-0: annn nnnn
 *     - nnn nnnn (Bits 6-0) represents the number of consecutive registers to read, minus one.
 *       - Example: To read 1 register, set `nnn nnnn` to `0000000`.
 *       - To read 2 registers, set `nnn nnnn` to `0000001`, and so on.
 *
 * - Reading Modes:
 *   - Single Register Read:
 *     - Set `nnn nnnn` to `0000000` to read a single register.
 *     - The response (register contents) is output in the frame following the command.
 *     - Instead of an acknowledgment word, the response contains the value of the specified register.
 *
 *   - Multiple Register Read:
 *     - If `nnn nnnn` is greater than zero, multiple registers are read sequentially.
 *     - The response occurs in the frame following the command, outputting the contents of each requested register.
 *     - Continue toggling the SCLK line to retrieve the entire data stream, as each register value is output sequentially.
 *     - Note: In multiple register reads, ADC conversion data are not output in the frame immediately following the command.
 *
 * Example:
 * - To read register 0x05 and 0x06, the command word would be:
 *   - 1010 1010 0000 0001 (where `a aaaa a` is `001010` for register 0x05, and `nnn nnnn` is `0000001` for 2 registers).
 */
static HAL_StatusTypeDef _register_read(uint16_t reg, uint8_t len, uint16_t *p_data) {
    uint8_t read_len = 0;
    uint16_t send_data[ADS130B04_SEND_CMD_TX_LEN];
    int16_t read_data[ADS130B04_SPI_WORD_MAX_BUFF_SIZE] = {
        0x00, };

    if (len > ADS130B04_CMD_RREG_MAX_LEN) {
        read_len = ADS130B04_CMD_RREG_MAX_LEN - 1;
        SYS_LOG_WARN("Read len changed: %d -> %d", read_len, ADS130B04_CMD_RREG_MAX_LEN);
    }
    else if (len == 0) {
        read_len = 0;
        SYS_LOG_WARN("Read len changed: %d -> %d", read_len, 1);
    }
    else {
        read_len = len - 1;
    }
    
    send_data[ADS130B04_SEND_CMD_IDX_CMD] = ADS130B04_CMD_RREG_HEAD;
    send_data[ADS130B04_SEND_CMD_IDX_CMD] |= BF_VAL(reg, ADS130B04_CMD_W_R_REG_ADDR_BCNT, ADS130B04_CMD_W_R_REG_ADDR_BOFF);
    send_data[ADS130B04_SEND_CMD_IDX_CMD] |= BF_VAL(read_len, ADS130B04_CMD_W_R_DATA_NUM_BCNT, ADS130B04_CMD_W_R_DATA_NUM_BOFF);
    if (ADS130B04_CRC_ENABLE == ads130b04_word_size.crc_enable) {
        send_data[ADS130B04_SEND_CMD_IDX_CRC] = _make_crc(&send_data[ADS130B04_SEND_CMD_IDX_CMD]);
        return HAL_ERROR;
    }
    else {
        send_data[ADS130B04_SEND_CMD_IDX_CRC] = 0; /* CRC */
    }
    
    SYS_VERIFY_SUCCESS(_comm_tx(&send_data[0], ADS130B04_SEND_CMD_TX_LEN));
    SYS_VERIFY_SUCCESS(_comm_rx(&read_data[0], len));
    
    if (len > 1) {
        /* [ACK] [REG DATA 1 (2Byte)] [REG DATA 2 (2Byte)] ... [REG DATA N (2Byte)] [CRC] */
        memcpy(p_data, &read_data[1], 2 * (len + 2));
    }
    else {
        /* [REG DATA] */
        memcpy(p_data, &read_data[0], 2);
    }
    
    return HAL_OK;
}

#define USE_SPI_TXRX_FUNC	0
static HAL_StatusTypeDef _comm_tx_rx(uint16_t *p_tx_data, uint8_t tx_len, int16_t *p_rx_data, uint8_t rx_len) {
    uint8_t tx_buff[ADS130B04_SPI_WORD_MIN_BUFF_SIZE] = {
        0, };
    uint8_t rx_buff[ADS130B04_SPI_WORD_MAX_BUFF_SIZE] = {
        0, };
    uint8_t word_size = ads130b04_word_size.byte_num;
    
    SYS_VERIFY_PARAM_NOT_NULL(ads130b04_context.spi_handle);
    /* Make Packet */
    _make_word_packet(p_tx_data, tx_len, &tx_buff[0]);
    
    /* Send Data */
    ADS130B04_SPI_Select();
    
#if USE_SPI_TXRX_FUNC == 1
    SYS_VERIFY_SUCCESS(HAL_SPI_TransmitReceive(ads130b04_context.spi_handle, &tx_buff[0], &rx_buff[0], word_size * tx_len, 500));
#else
    SYS_VERIFY_SUCCESS(HAL_SPI_Transmit(ads130b04_context.spi_handle, &tx_buff[0], word_size * tx_len, 500));
    ADS130B04_SPI_Deselect();
#endif
    
#if USE_SPI_TXRX_FUNC == 1
#else
    ADS130B04_SPI_Select();
    SYS_VERIFY_SUCCESS(HAL_SPI_Receive(ads130b04_context.spi_handle, &rx_buff[0], word_size * rx_len, 500));
#endif
    
    ADS130B04_SPI_Deselect();
    
    /* Parse Packet */
    _parse_word_packet(&rx_buff[0], rx_len, p_rx_data);
    
    return HAL_OK;
}

static HAL_StatusTypeDef _comm_tx(uint16_t *p_data, uint8_t len) {
    uint8_t tx_buff[ADS130B04_SPI_WORD_MIN_BUFF_SIZE] = {
        0, };
    uint8_t word_size = ads130b04_word_size.byte_num;
    
    SYS_VERIFY_PARAM_NOT_NULL(ads130b04_context.spi_handle);
    
    /* Make Packet */
    _make_word_packet(p_data, len, &tx_buff[0]);
    
    /* Send Data */
    ADS130B04_SPI_Select();
    SYS_VERIFY_SUCCESS(HAL_SPI_Transmit(ads130b04_context.spi_handle, &tx_buff[0], word_size * len, 500));
    ADS130B04_SPI_Deselect();
    
    return HAL_OK;
}

static HAL_StatusTypeDef _comm_rx(int16_t *p_data, uint8_t len) {
    uint8_t rx_buff[ADS130B04_SPI_WORD_MAX_BUFF_SIZE] = {
        0, };
    uint8_t word_size = ads130b04_word_size.byte_num;
    
    SYS_VERIFY_PARAM_NOT_NULL(ads130b04_context.spi_handle);
    
    /* Read Data */
    ADS130B04_SPI_Select();
    SYS_VERIFY_SUCCESS(HAL_SPI_Receive(ads130b04_context.spi_handle, &rx_buff[0], word_size * len, 500));
    ADS130B04_SPI_Deselect();
    
    /* Parse Packet */
    _parse_word_packet(&rx_buff[0], len, p_data);
    
    return HAL_OK;
}

static void _make_word_packet(uint16_t *p_data, uint8_t len, uint8_t *p_result) {
    uint8_t word_size = ads130b04_word_size.byte_num;
    
    for (uint8_t idx = 0; idx < len; idx++) {
        p_result[word_size * idx] = (p_data[idx] >> 8) & 0xFF;
        p_result[word_size * idx + 1] = p_data[idx] & 0xFF;
        
        /* padding.. */
    }
}

static void _parse_word_packet(uint8_t *p_data, uint8_t len, int16_t *p_result) {
    uint8_t word_size = ads130b04_word_size.byte_num;
    
    for (uint8_t idx = 0; idx < len; idx++) {
        p_result[idx] = (((p_data[word_size * idx] << 8) & 0xFF00) | (p_data[word_size * idx + 1] & 0xFF));
    }
}

static uint16_t _make_crc(uint16_t *p_data) {
    SYS_LOG_ERR("CRC Mode is not developed");
    
    return 0;
}

/*
 * MUX Channel Selection Table
 *
 * EN   A1   A0   | Selected Input Connected to Drain (D) Pin
 * ----------------------------------------------------------
 *  0    X    X   | All channels are off
 *  1    0    0   | S1: MP_SIG_CH_1
 *  1    0    1   | S2: MP_SIG_CH_2
 *  1    1    0   | S3: MP_SIG_CH_3
 *  1    1    1   | S4: DAC_CHECK
 *
 * - EN: Enable bit, when set to 0, all channels are off regardless of A1 and A0.
 * - A1, A0: Address bits to select the channel when EN is 1.
 * - X: Don’t care; the value of this bit doesn't matter when EN is 0.
 * - S1, S2, S3, S4: Represents the channels that can be selected.
 */
static void _ch4_mux_enable(bool enable) {
    if (enable) {
        HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(M_SEL_EN_GPIO_Port, M_SEL_EN_Pin, GPIO_PIN_RESET);
    }
}

static void _ch4_mux_select(Ads130b04Ch3MuxCh_t ch) {
    SYS_VERIFY_TRUE_VOID(ch < DRV_ADS130B04_MUX_CH_MAX);
    ads130b04_context.cur_mux_ch = ch;
    
    switch (ch) {
        case DRV_ADS130B04_MUX_CH_0:
            HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
            break;
            
        case DRV_ADS130B04_MUX_CH_1:
            HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_SET);
            break;
            
        case DRV_ADS130B04_MUX_CH_2:
            HAL_GPIO_WritePin(M_SEL_A1_GPIO_Port, M_SEL_A1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M_SEL_A0_GPIO_Port, M_SEL_A0_Pin, GPIO_PIN_RESET);
            break;
            
        default:
            SYS_LOG_ERR("Invalid Monitor Ch Selected: %d", ch);
            return;
    }

    SYS_LOG_INFO("Monitor Ch %d Selected", ch);
}

