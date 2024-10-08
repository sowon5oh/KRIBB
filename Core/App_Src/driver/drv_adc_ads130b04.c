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
#include "main.h"
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
    /* SPS: Samples per seconds */
    ADS130B04_OSR_MODE_128 = 0, /* 32 	 ksps */
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
    ads130b04SpiWordSize_t size_id;
    uint8_t byte_num;
} ads130b04SpiWordCfg_t;

typedef enum {
    ADS130B04_CH_GAIN_MODE_1 = 0, /* +/- 1.2     V */
    ADS130B04_CH_GAIN_MODE_2, /* +/- 600    mV */
    ADS130B04_CH_GAIN_MODE_4, /* +/- 300    mV */
    ADS130B04_CH_GAIN_MODE_8, /* +/- 150    mV */
    ADS130B04_CH_GAIN_MODE_16, /* +/-  75    mV */
    ADS130B04_CH_GAIN_MODE_32, /* +/-  37.5  mV */
    ADS130B04_CH_GAIN_MODE_64, /* +/-  18.75 mV */
    ADS130B04_CH_GAIN_MODE_128, /* +/-  9.375 mV */
} ads130b04ChGainMode_t;

typedef enum {
    DRV_ADS130B04_CH_INPUT_MODE_CONN = 0,
    DRV_ADS130B04_CH_INPUT_MODE_DISCONN,
    DRV_ADS130B04_CH_INPUT_MODE_TEST_POSITIVE,
    DRV_ADS130B04_CH_INPUT_MODE_TEST_NEGATIVE,
} ads130b04ChInputMode_t;

typedef struct {
    bool enable;
    ads130b04ChGainMode_t gain_mode;
    ads130b04ChInputMode_t input_mode;
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

    ads130b04StateMode_t state;

    ads130b04Data_t ads130b04_cfg;

    /* ADS130B04 Settings */
    bool lock;
    bool global_chop;
    ads130b04SyncMode_t sync_mode;
    ads130b04SpiWordCfg_t word_size;

    /* ADS130B04 Clock Config */
    ads130b04ClockSel_t clock_sel;
    ads130b04OsrMode_t osr_mode;
    ads130b04PwrMode_t pwr_mode;
    ads130b04ChCfg_t ch_cfg[DRV_ADS130B04_CH_NUM];

    /* ADC Average Request & Callback */
    bool ads130b04_read;

    uint8_t ads130b04_ch;
    uint16_t ads130b04_num;
    uint16_t ads130b04_cnt;
    uint16_t ads130b04_wait_time;

    double ads130b04_sum;

    MeasAvrResultCb_t cb_fn;
} ads130b04Context_t;

/* Private define ------------------------------------------------------------*/
#define ADS130B04_SPI_Select()    		    HAL_GPIO_WritePin(ADC_CS__GPIO_Port, ADC_CS__Pin, GPIO_PIN_RESET)
#define ADS130B04_SPI_Deselect()  		    HAL_GPIO_WritePin(ADC_CS__GPIO_Port, ADC_CS__Pin, GPIO_PIN_SET)

#define ADS130B04_CMD_NULL				    0x0000	/*  */
#define ADS130B04_CMD_RESET				    0x0011	/*  */
#define ADS130B04_CMD_STANDBY				    0x0022	/* places the device in a low-power standby mode */
#define ADS130B04_CMD_WAKEUP				    0x0033	/* returns the device to conversion mode from standby mode */
#define ADS130B04_CMD_LOCK				    0x0555	/*  */
#define ADS130B04_CMD_UNLOCK				    0x0655	/*  */

#define ADS130B04_CMD_RESP_RESET			    0xFF54
#define ADS130B04_CMD_RESP_STANDBY		    ADS130B04_CMD_STANDBY
#define ADS130B04_CMD_RESP_WAKEUP			    ADS130B04_CMD_WAKEUP
#define ADS130B04_CMD_RESP_LOCK			    ADS130B04_CMD_LOCK
#define ADS130B04_CMD_RESP_UNLOCK			    ADS130B04_CMD_UNLOCK

#define ADS130B04_CMD_RREG_HEAD			    0xA000	/* used to read the device registers */
#define ADS130B04_CMD_WREG_HEAD			    0x6000
#define ADS130B04_CMD_W_R_REG_ADDR_BOFF	    7
#define ADS130B04_CMD_W_R_REG_ADDR_BCNT	    6
#define ADS130B04_CMD_W_R_DATA_NUM_BOFF	    0
#define ADS130B04_CMD_W_R_DATA_NUM_BCNT	    7
#define ADS130B04_CMD_RREG_MAX_LEN		    10

/* DEVICE SETTINGS AND STATUS INDICATORS (Read-Only Registers) */
#define ADS130B04_REG_ADDR_ID				    0x00
#define ADS130B04_REG_ADDR_STATUS			    0x01
/* GLOBAL SETTINGS ACROSS CHANNELS */
#define ADS130B04_REG_ADDR_MODE				0x02
#define ADS130B04_REG_ADDR_CLOCK				0x03
#define ADS130B04_PWR_MODE_BOFF				0
#define ADS130B04_PWR_MODE_BCNT				2
#define ADS130B04_OSR_MODE_BOFF				2
#define ADS130B04_OSR_MODE_BCNT				3
#define ADS130B04_CLK_SEL_BOFF			    7
#define ADS130B04_CLK_SEL_BCNT			    1
#define DRV_ADS130B04_CH0_EN_BOFF				    8
#define DRV_ADS130B04_CH0_EN_BCNT				    1
#define DRV_ADS130B04_CH1_EN_BOFF				    9
#define DRV_ADS130B04_CH1_EN_BCNT				    1
#define DRV_ADS130B04_CH2_EN_BOFF				    10
#define DRV_ADS130B04_CH2_EN_BCNT				    1
#define DRV_ADS130B04_CH3_EN_BOFF				    11
#define DRV_ADS130B04_CH3_EN_BCNT				    1
#define ADS130B04_REG_ADDR_GAIN				0x04
#define ADS130B04_GAIN_CFG_CH0_BOFF			0
#define ADS130B04_GAIN_CFG_CH0_BCNT			3
#define ADS130B04_GAIN_CFG_CH1_BOFF			4
#define ADS130B04_GAIN_CFG_CH1_BCNT			3
#define ADS130B04_GAIN_CFG_CH2_BOFF			8
#define ADS130B04_GAIN_CFG_CH2_BCNT			3
#define ADS130B04_GAIN_CFG_CH3_BOFF			12
#define ADS130B04_GAIN_CFG_CH3_BCNT			3
#define ADS130B04_REG_ADDR_GLOBAL_CHOP_CFG	0x06
/* CHANNEL-SPECIFIC SETTINGS */
#define ADS130B04_REG_ADDR_CH0_CFG			0x09 //9
#define ADS130B04_REG_ADDR_CH1_CFG			0x0E //14
#define ADS130B04_REG_ADDR_CH2_CFG			0x13 //19
#define ADS130B04_REG_ADDR_CH3_CFG			0x18 //24
#define DRV_ADS130B04_CH_CFG_MUX_BOFF			    1
#define DRV_ADS130B04_CH_CFG_MUX_BCNT			    2
/* REGISTER MAP CRC REGISTER (Read-Only Register) */
#define ADS130B04_REG_ADDR_REG_MAP_CRC		0x3E //62

#define ADS130B04_SPI_WORD_BYTE_NUM_16BIT	    2
#define ADS130B04_SPI_WORD_BYTE_NUM_24BIT	    3
#define ADS130B04_SPI_WORD_BYTE_NUM_32BIT	    4

#define ADS130B04_SPI_CMD_BYTE_SIZE	        2       /* 16 bit */

#define ADS130B04_SPI_WORD_MIN_BUFF_SIZE		ADS130B04_SPI_WORD_BYTE_NUM_32BIT * 6  /* Command, Crc */
#define ADS130B04_SPI_WORD_MAX_BUFF_SIZE		ADS130B04_SPI_WORD_MIN_BUFF_SIZE * 5   /* Command, Crc, Data... */
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void _ads130b04DataReset(void);
HAL_StatusTypeDef _ads130b04SendCmd(ads130b04CmdId_t cmd);
HAL_StatusTypeDef _ads130b04GetData(bool first_read);
HAL_StatusTypeDef _ads130b04GetData2(bool first_read);
HAL_StatusTypeDef _ads130b04SetClockCfg(void);
HAL_StatusTypeDef _ads130b04SetGainCfg(void);
HAL_StatusTypeDef _ads130b04SetChMuxCfg(void);

HAL_StatusTypeDef _ads130b04WriteRegister(uint16_t reg, uint16_t data);
HAL_StatusTypeDef _ads130b04ReadRegister(uint16_t reg, uint8_t len, uint16_t *p_data);

HAL_StatusTypeDef _ads130b04TransmitReceive(uint16_t *p_tx_data, int16_t *p_rx_data, uint8_t len);
HAL_StatusTypeDef _ads130b04Transmit(uint16_t *p_data, uint8_t len);
HAL_StatusTypeDef _ads130b04Receive(int16_t *p_data, uint8_t len);

void _makeWordPacket(uint16_t *p_data, uint8_t len, uint8_t *p_result);
void _parseWordPacket(uint8_t *p_data, uint8_t len, int16_t *p_result);
/* Private variables ---------------------------------------------------------*/
ads130b04Context_t ads130b04_context = {
    .ads130b04_read = false,
    .ads130b04_num = 0,
    .ads130b04_cnt = 0,
    .ads130b04_wait_time = 0,
    .word_size.byte_num = ADS130B04_SPI_WORD_BYTE_NUM_24BIT };

uint16_t ads130b04_avr_buff[DRV_ADS130B04_CH_NUM][ADS130B04_AVR_BUFF_MAX_NUM];

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_ADS130B04_Init(SPI_HandleTypeDef *p_hdl) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint16_t read_data = 0;

    /* attach spi handle */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    ads130b04_context.spi_handle = (SPI_HandleTypeDef*) p_hdl;

    /* Pin Reset */
    HAL_GPIO_WritePin(ADC_RST__GPIO_Port, ADC_RST__Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(ADC_RST__GPIO_Port, ADC_RST__Pin, GPIO_PIN_SET);

    /* General Config */
    ads130b04_context.state = ADS130B04_STATE_STANDBY;
    ads130b04_context.lock = false;
    ads130b04_context.global_chop = true;
    ads130b04_context.sync_mode = ADS130B04_SYNC_MODE_OFF;
    ads130b04_context.word_size.byte_num = ADS130B04_SPI_WORD_BYTE_NUM_24BIT;

    /* Clock Config */
    ads130b04_context.clock_sel = ADS130B04_CLOCK_INTERNAL_OSC;
    ads130b04_context.osr_mode = ADS130B04_OSR_MODE_1024;
    ads130b04_context.pwr_mode = ADS130B04_PWR_MODE_HIGH_RESOLUTION;

    /* Reset */
    ret = _ads130b04SendCmd(ADS130B04_CMD_ID_RESET);
    if (ret == HAL_OK) {
        ret = _ads130b04SendCmd(ADS130B04_CMD_ID_STANDBY);
    }

    /* Read ID */
    ret = _ads130b04ReadRegister(ADS130B04_REG_ADDR_ID, 1, &read_data);
    if (ret == HAL_OK) {
        SYS_LOG_INFO("ADC ID: %04X", read_data); //5404 - 21508
    }

    ret = _ads130b04ReadRegister(ADS130B04_REG_ADDR_STATUS, 1, &read_data);
    if (ret == HAL_OK) {
        SYS_LOG_INFO("ADC STATUS: %04X", read_data); //0500 - 1280
    }

    ret = _ads130b04ReadRegister(ADS130B04_REG_ADDR_MODE, 1, &read_data);
    if (ret == HAL_OK) {
        SYS_LOG_INFO("ADC MODE: %04X", read_data); //0510 - 1296
    }

    ret = _ads130b04ReadRegister(ADS130B04_REG_ADDR_CLOCK, 1, &read_data);
    if (ret == HAL_OK) {
        SYS_LOG_INFO("ADC CLOCK: %04X", read_data); //0F8E - 3970
    }

    /* Standby */
    ret = _ads130b04SendCmd(ADS130B04_CMD_ID_STANDBY);

    /* Initialize */
    ads130b04_context.ads130b04_num = 0;
    ads130b04_context.ads130b04_cnt = 0;
    ads130b04_context.ads130b04_sum = 0;

    /* Ch0 Config */
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_0].enable = true;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_0].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_0].gain_mode = ADS130B04_CH_GAIN_MODE_1;

    /* Ch1 Config */
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_1].enable = true;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_1].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_1].gain_mode = ADS130B04_CH_GAIN_MODE_1;

    /* Ch2 Config */
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_2].enable = true;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_2].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_2].gain_mode = ADS130B04_CH_GAIN_MODE_1;

    /* Ch3 Config */
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_3].enable = true;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_3].input_mode = DRV_ADS130B04_CH_INPUT_MODE_CONN;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_3].gain_mode = ADS130B04_CH_GAIN_MODE_1;

    /* Apply Config */
    _ads130b04SetClockCfg();
    _ads130b04SetGainCfg();
    _ads130b04SetChMuxCfg();

    return ret;
}

HAL_StatusTypeDef DRV_ADS130B04_Start(void) {
    HAL_StatusTypeDef ret = HAL_OK;

    /* Wake Up */
    ret = _ads130b04SendCmd(ADS130B04_CMD_ID_WAKEUP);

    return ret;
}

void DRV_ADS130B04_CFG_Change(ads130b04Data_t *p_req_info) {
    HAL_StatusTypeDef ret = HAL_OK;

    ads130b04_context.ads130b04_cfg.Mux_ch = p_req_info->Mux_ch;
    ads130b04_context.ads130b04_cfg.ch = p_req_info->ch;
    ads130b04_context.osr_mode = p_req_info->sps; //sps
    ads130b04_context.ads130b04_num = p_req_info->ads130b04_num; //samples
    ads130b04_context.ads130b04_wait_time = p_req_info->wait_time;

    //ret = _ads130b04SetClockCfg();

    if (ret != HAL_OK) {
        SYS_LOG_ERR("ADC Clock Setting failed");
    }
}

HAL_StatusTypeDef DRV_ADS130B04_Stop(void) {
    HAL_StatusTypeDef ret = HAL_OK;

    /* Standby */
    ret = _ads130b04SendCmd(ADS130B04_CMD_ID_STANDBY);

    /* Reset measure data */
    _ads130b04DataReset();

    return ret;
}

void DRV_ADS130B04_MeasStop(void) {
    HAL_StatusTypeDef ret = HAL_OK;

    /* Reset measure data */
    _ads130b04DataReset();

    /* Disable ADC Ch */
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_0].enable = false;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_1].enable = false;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_2].enable = false;
    ads130b04_context.ch_cfg[DRV_ADS130B04_CH_3].enable = false;

    ret = _ads130b04SetClockCfg();

    if (ret != HAL_OK) {
        SYS_LOG_ERR("ADC Clock Setting failed");
    }
}

void DRV_ADS130B04_GetSetting(uint8_t *ch, uint8_t *sps, uint16_t *samples, uint16_t *wait_time) {
    *ch = ads130b04_context.ads130b04_cfg.ch;
    *sps = ads130b04_context.osr_mode;
    *samples = ads130b04_context.ads130b04_num;
    *wait_time = ads130b04_context.ads130b04_wait_time;
}

HAL_StatusTypeDef DRV_ADS130B04_ReqAvr(ads130b04Data_t *p_req_info, MeasAvrResultCb_t cb_fn) {
    HAL_StatusTypeDef ret = HAL_OK;
    bool clock_recfg = false;

    if ((p_req_info == NULL) || (cb_fn == NULL)) {
        ret = HAL_ERROR;
        SYS_LOG_ERR("Invalid Parameter");
    }
    else {
        ads130b04_context.ads130b04_cfg.ch = p_req_info->ch;
        ads130b04_context.ads130b04_num = p_req_info->ads130b04_num;
        ads130b04_context.ads130b04_cnt = 0;
        ads130b04_context.cb_fn = cb_fn;

        if (ads130b04_context.ch_cfg[ads130b04_context.ads130b04_cfg.ch].enable != true) {
            SYS_LOG_INFO("Enable CH %d", ads130b04_context.ads130b04_cfg.ch);
            ads130b04_context.ch_cfg[ads130b04_context.ads130b04_cfg.ch].enable = true;
            clock_recfg = true;
        }

        if (ads130b04_context.osr_mode != (ads130b04OsrMode_t) p_req_info->sps) {
            SYS_LOG_INFO("Change OSR (SPS) %d ==> %d", ads130b04_context.osr_mode, p_req_info->sps);
            ads130b04_context.osr_mode = p_req_info->sps;

            clock_recfg = true;
        }

        if (clock_recfg == true) {
            ret = _ads130b04SetClockCfg();
        }

        if (ret == HAL_OK) {
            /* Get first datay to init FIFO for data ready signal */
            SYS_LOG_DEBUG("ADC Init FIFO");
            _ads130b04GetData(true);
            _ads130b04GetData(true);

            /* Read Start */
            SYS_LOG_DEBUG("ADC Read Start");
            ads130b04_context.ads130b04_read = true;
        }
        else {
            SYS_LOG_ERR("ADC Meas Setting failed");
        }
    }

    return ret;
}

/* Private user code ---------------------------------------------------------*/
void _ads130b04DataReset(void) {
    ads130b04_context.ads130b04_read = false;
    ads130b04_context.ads130b04_cnt = 0;
    ads130b04_context.ads130b04_sum = 0;
}

HAL_StatusTypeDef _ads130b04SendCmd(ads130b04CmdId_t cmd) {
    HAL_StatusTypeDef ret = HAL_OK;

    uint16_t tx_buff[6] = {
        0x00, };
    int16_t rx_buff[6] = {
        0x00, };

    uint16_t send_cmd = 0x0000;
    uint16_t cmd_resp = 0;
    uint16_t crc = 0;

    switch (cmd) {
        case ADS130B04_CMD_ID_RESET:
            send_cmd = ADS130B04_CMD_RESET;
            cmd_resp = ADS130B04_CMD_RESP_RESET;
            SYS_LOG_INFO("ADC Send Command Reset: %04X", send_cmd); //0011 - 17
            break;

        case ADS130B04_CMD_ID_STANDBY:
            send_cmd = ADS130B04_CMD_STANDBY;
            cmd_resp = ADS130B04_CMD_RESP_STANDBY;
            ads130b04_context.state = ADS130B04_STATE_STANDBY;
            SYS_LOG_INFO("ADC Send Command Standby: %04X", send_cmd); // 0022 - 34
            break;

        case ADS130B04_CMD_ID_WAKEUP:
            send_cmd = ADS130B04_CMD_WAKEUP;
            cmd_resp = ADS130B04_CMD_RESP_WAKEUP;
            if (ads130b04_context.global_chop == false) {
                ads130b04_context.state = ADS130B04_STATE_CONTINOUS;
            }
            else {
                ads130b04_context.state = ADS130B04_STATE_GLOBAL_CHOP;
            }
            SYS_LOG_INFO("ADC Send Command Wakeup: %04X", send_cmd); //
            break;

        case ADS130B04_CMD_ID_LOCK:
            send_cmd = ADS130B04_CMD_LOCK;
            cmd_resp = ADS130B04_CMD_RESP_LOCK;
            ads130b04_context.lock = true;
            SYS_LOG_INFO("ADC Send Command Lock: %04X", send_cmd);
            break;

        case ADS130B04_CMD_ID_UNLOCK:
            send_cmd = ADS130B04_CMD_UNLOCK;
            cmd_resp = ADS130B04_CMD_RESP_UNLOCK;
            ads130b04_context.lock = false;
            SYS_LOG_INFO("ADC Send Command Unlock: %04X", send_cmd);
            break;

        default:
            ret = HAL_ERROR;
            SYS_LOG_INFO("Not supported command %04X", send_cmd);
            break;
    }

    /*
     * Make CMD Array: [CMD] [CRC]
     * */
    tx_buff[0] = send_cmd;
    tx_buff[1] = crc;

    if (ret == HAL_OK) {
        ret = _ads130b04TransmitReceive(&tx_buff[0], &rx_buff[0], 6);
    }

    if (ret == HAL_OK) {
        if (cmd_resp == rx_buff[0]) {
            ret = HAL_OK;
            SYS_LOG_INFO("ADC Command valid response: %04X", rx_buff[0]);
        }
        else {
            ret = HAL_ERROR;
            SYS_LOG_ERR("ADC Command wrong response: %04X", rx_buff[0]);
        }
    }
    else {
        SYS_LOG_ERR("ADC Command response receive failed");
    }

    return ret;
}

HAL_StatusTypeDef _ads130b04GetData(bool first_read) {
    HAL_StatusTypeDef ret = HAL_OK;

    uint16_t tx_buff[6] = {
        0x00, };
    int16_t rx_buff[6] = {
        0x00, };
    static float lsb = 0;
    static uint8_t data_idx = 0;
    float ads130b04_result = 0;
    int32_t ads130b04_result2 = 0;
    float ads130b04_avr = 0;
    int32_t ads130b04_avr2 = 0;

    tx_buff[0] = ADS130B04_CMD_NULL;
    ret = _ads130b04TransmitReceive(&tx_buff[0], &rx_buff[0], 6); //3

    if (ret == HAL_OK) {
        SYS_LOG_INFO("Get Ch Data: %04X %04X %04X %04X %04X", rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4]); /* [STATUS] [CH0 DATA] [CH1 DATA] */

        if (first_read == false) {
            ads130b04_result = (float) ((rx_buff[data_idx]) * lsb); //(int16_t)
            ads130b04_result2 = (int32_t) ((rx_buff[data_idx]) * lsb * 1000); //(int16_t)
            SYS_LOG_INFO("ADC Result: %d mV", ads130b04_result2);

            ads130b04_context.ads130b04_sum += ads130b04_result;

            if (++ads130b04_context.ads130b04_cnt >= ads130b04_context.ads130b04_num) {
                ads130b04_avr = (float) ads130b04_context.ads130b04_sum / ads130b04_context.ads130b04_cnt;
                ads130b04_avr2 = (int32_t) ((ads130b04_context.ads130b04_sum / ads130b04_context.ads130b04_cnt) * 1000);
                SYS_LOG_INFO("ADC Average: %d mV", ads130b04_avr2);
                ads130b04_context.ads130b04_sum = 0;

                ads130b04_context.cb_fn(ads130b04_avr);
                ads130b04_context.ads130b04_read = false;
                ads130b04_context.ads130b04_cnt = 0;
            }
        }
        else {
            lsb = (float) 1200 / pow(2, ads130b04_context.ch_cfg[ads130b04_context.ads130b04_cfg.ch].gain_mode) / pow(2, 15);
            if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_0) {
                data_idx = 1;
            }
            else if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_1) {
                data_idx = 2;
            }
            else if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_2) {
                data_idx = 3;
            }
            else if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_3) {
                data_idx = 4;
            }
        }
    }
    else {
        SYS_LOG_ERR("Read Status and ADC Result failed");
    }

    return ret;
}

/*
 HAL_StatusTypeDef _ads130b04GetData2(bool first_read)
 {
 HAL_StatusTypeDef ret = HAL_OK;

 uint16_t tx_buff[6] = {0x00,};
 uint16_t rx_buff[6] = {0x00,};
 static float lsb = 0;
 static uint8_t data_idx = 0;
 float ads130b04_result = 0;
 int32_t ads130b04_result2 = 0;
 float ads130b04_avr = 0;
 int32_t ads130b04_avr2 = 0;

 tx_buff[0] = ADS130B04_CMD_NULL;
 ret = _ads130b04TransmitReceive(&tx_buff[0], &rx_buff[0], 6); //3

 if (ret == HAL_OK)
 {
 SYS_LOG_INFO("Get Ch Data: %04X %04X %04X %04X %04X", rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4]); // [STATUS] [CH0 DATA] [CH1 DATA]

 if (first_read == false)
 {
 ads130b04_result = (float)((rx_buff[data_idx]) * lsb); //(int16_t)
 ads130b04_result2 = (int32_t)((rx_buff[data_idx]) * lsb * 1000); //(int16_t)
 SYS_LOG_INFO("ADC Result: %d mV", ads130b04_result2);

 ads130b04_context.ads130b04_sum += ads130b04_result;

 if (++ads130b04_context.ads130b04_cnt >= ads130b04_context.ads130b04_num)
 {
 ads130b04_avr = (float)ads130b04_context.ads130b04_sum / ads130b04_context.ads130b04_cnt;
 ads130b04_avr2 = (int32_t)((ads130b04_context.ads130b04_sum / ads130b04_context.ads130b04_cnt)*1000);
 SYS_LOG_INFO("ADC Average: %d mV", ads130b04_avr2);
 ads130b04_context.ads130b04_sum = 0;

 ads130b04_context.cb_fn(ads130b04_avr);
 ads130b04_context.ads130b04_read = false;
 ads130b04_context.ads130b04_cnt = 0;
 }
 }
 else
 {
 lsb = (float)2400 / pow(2, ads130b04_context.ch_cfg[ads130b04_context.ads130b04_cfg.ch].gain_mode) / pow(2, 16);
 if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_0)
 {
 data_idx = 1;
 }
 else if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_1)
 {
 data_idx = 2;
 }
 else if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_2)
 {
 data_idx = 3;
 }
 else if (ads130b04_context.ads130b04_cfg.ch == DRV_ADS130B04_CH_3)
 {
 data_idx = 4;
 }
 }
 }
 else
 {
 SYS_LOG_ERR("Read Status and ADC Result failed");
 }

 return ret;
 }
 */

HAL_StatusTypeDef _ads130b04SetClockCfg(void) {
    HAL_StatusTypeDef ret = HAL_OK;

    uint16_t send_cfg = 0x00;

    /* Ch Enable */
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_0].enable, DRV_ADS130B04_CH0_EN_BCNT, DRV_ADS130B04_CH0_EN_BOFF);
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_1].enable, DRV_ADS130B04_CH1_EN_BCNT, DRV_ADS130B04_CH1_EN_BOFF);
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_2].enable, DRV_ADS130B04_CH2_EN_BCNT, DRV_ADS130B04_CH2_EN_BOFF);
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_3].enable, DRV_ADS130B04_CH3_EN_BCNT, DRV_ADS130B04_CH3_EN_BOFF);

    /* Clock Select */
    send_cfg |= BF_VAL(ads130b04_context.clock_sel, ADS130B04_CLK_SEL_BCNT, ADS130B04_CLK_SEL_BOFF);

    /* OSR Config */
    send_cfg |= BF_VAL(ads130b04_context.osr_mode, ADS130B04_OSR_MODE_BCNT, ADS130B04_OSR_MODE_BOFF);

    /* Power mode Config */
    send_cfg |= BF_VAL(ads130b04_context.pwr_mode, ADS130B04_PWR_MODE_BCNT, ADS130B04_PWR_MODE_BOFF);

    ret = _ads130b04WriteRegister(ADS130B04_REG_ADDR_CLOCK, send_cfg);

    if (ret == HAL_OK) {
        SYS_LOG_INFO("ADC Set Clock Config Success: %04X", send_cfg); //0F0E - 3854
    }
    else {
        SYS_LOG_INFO("ADC Set Clock Config Failed");
    }

    return ret;
}

HAL_StatusTypeDef _ads130b04SetGainCfg(void) {
    HAL_StatusTypeDef ret = HAL_OK;

    uint16_t send_cfg = 0x00;

    /* Clock Config */
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_0].gain_mode, ADS130B04_GAIN_CFG_CH0_BCNT, ADS130B04_GAIN_CFG_CH0_BOFF);
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_1].gain_mode, ADS130B04_GAIN_CFG_CH1_BCNT, ADS130B04_GAIN_CFG_CH1_BOFF);
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_2].gain_mode, ADS130B04_GAIN_CFG_CH2_BCNT, ADS130B04_GAIN_CFG_CH2_BOFF);
    send_cfg |= BF_VAL(ads130b04_context.ch_cfg[DRV_ADS130B04_CH_3].gain_mode, ADS130B04_GAIN_CFG_CH3_BCNT, ADS130B04_GAIN_CFG_CH3_BOFF);

    ret = _ads130b04WriteRegister(ADS130B04_REG_ADDR_GAIN, send_cfg);

    if (ret == HAL_OK) {
        SYS_LOG_INFO("ADC Set Gain Config Success: %04X", send_cfg); //0000
    }
    else {
        SYS_LOG_INFO("ADC Set Gain Config Failed");
    }

    return ret;
}

HAL_StatusTypeDef _ads130b04SetChMuxCfg(void) {
    HAL_StatusTypeDef ret = HAL_OK;

    uint16_t send_cfg = 0x00;
    uint8_t input_mode[DRV_ADS130B04_CH_NUM] = {
        0, };

    input_mode[0] = ads130b04_context.ch_cfg[DRV_ADS130B04_CH_0].input_mode;
    input_mode[1] = ads130b04_context.ch_cfg[DRV_ADS130B04_CH_1].input_mode;
    input_mode[2] = ads130b04_context.ch_cfg[DRV_ADS130B04_CH_2].input_mode;
    input_mode[3] = ads130b04_context.ch_cfg[DRV_ADS130B04_CH_3].input_mode;

    /* Ch MUX Config */
    for (uint8_t idx = 0; idx < 4; idx++) //5
        {
        send_cfg = BF_VAL(input_mode[idx], DRV_ADS130B04_CH_CFG_MUX_BCNT, DRV_ADS130B04_CH_CFG_MUX_BOFF);
        ret = _ads130b04WriteRegister(ADS130B04_REG_ADDR_CH0_CFG + 5 * idx, send_cfg);

        if (ret == HAL_OK) {
            SYS_LOG_INFO("ADC Set MUX CH%d Config Success: %04X", idx, send_cfg); // CH1234 success : 0000
        }
        else {
            SYS_LOG_ERR("ADC Set MUX CH%d Config Failed");
            break;
        }
    }

    return ret;
}

HAL_StatusTypeDef _ads130b04WriteRegister(uint16_t reg, uint16_t data) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint16_t send_data[3] = {
        0x00, };
    int16_t resp_data = 0x00;
    uint16_t save_data = 0x00;
    uint16_t resp_chk;

    send_data[0] = ADS130B04_CMD_WREG_HEAD;
    send_data[0] |= BF_VAL(reg, ADS130B04_CMD_W_R_REG_ADDR_BCNT, ADS130B04_CMD_W_R_REG_ADDR_BOFF);
    send_data[1] = data;

    ret = _ads130b04Transmit(&send_data[0], 3);

    if (ret == HAL_OK) {
        ret = _ads130b04Receive(&resp_data, 1);
    }

    if (ret == HAL_OK) {
        resp_chk = BF_GET(resp_data, ADS130B04_CMD_W_R_REG_ADDR_BCNT, ADS130B04_CMD_W_R_REG_ADDR_BOFF);
        if (reg != resp_chk) {
            SYS_LOG_ERR("Register Write failed, cmd don't match: %04X | %04X", reg, resp_chk);

            ret = HAL_ERROR;
        }

        resp_chk = BF_GET(resp_data, ADS130B04_CMD_W_R_DATA_NUM_BCNT, ADS130B04_CMD_W_R_DATA_NUM_BOFF);
        if (resp_chk != 0) {
            SYS_LOG_ERR("Register Write failed, len don't match: 0 | %d", resp_chk);

            ret = HAL_ERROR;
        }
    }

    if (ret == HAL_OK) {
        ret = _ads130b04ReadRegister(reg, 1, &save_data);

        if (data != save_data) {
            SYS_LOG_ERR("Write & Read data don't match: %04X | %04X", data, save_data);

            ret = HAL_ERROR;
        }
    }

    return ret;
}

HAL_StatusTypeDef _ads130b04ReadRegister(uint16_t reg, uint8_t len, uint16_t *p_data) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t read_len = 0;
    uint16_t send_data[ADS130B04_SPI_WORD_MIN_BUFF_SIZE] = {
        0x00, };
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

    send_data[0] = ADS130B04_CMD_RREG_HEAD;
    send_data[0] |= BF_VAL(reg, ADS130B04_CMD_W_R_REG_ADDR_BCNT, ADS130B04_CMD_W_R_REG_ADDR_BOFF);
    send_data[0] |= BF_VAL(read_len, ADS130B04_CMD_W_R_DATA_NUM_BCNT, ADS130B04_CMD_W_R_DATA_NUM_BOFF);

    ret = _ads130b04Transmit(&send_data[0], 2);

    if (ret == HAL_OK) {
        ret = _ads130b04Receive(&read_data[0], len);
    }

    if (ret == HAL_OK) {
        if (len > 1) {
            /* [ACK] [DATA 1] [DATA 2] ... [DATA N] [CRC] */
            memcpy(p_data, &read_data[1], 2 * (len + 2));
        }
        else {
            memcpy(p_data, &read_data[0], 2);
        }
    }

    return ret;
}

#define USE_SPI_TXRX_FUNC	0
HAL_StatusTypeDef _ads130b04TransmitReceive(uint16_t *p_tx_data, int16_t *p_rx_data, uint8_t len) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t tx_buff[ADS130B04_SPI_WORD_MIN_BUFF_SIZE] = {
        0, };
    uint8_t rx_buff[ADS130B04_SPI_WORD_MAX_BUFF_SIZE] = {
        0, };
    uint8_t word_size = ads130b04_context.word_size.byte_num;

    if (ads130b04_context.spi_handle != NULL) {
        /* Make Packet */
        _makeWordPacket(p_tx_data, len, &tx_buff[0]);

        /* Send Data */
        ADS130B04_SPI_Select();
#if USE_SPI_TXRX_FUNC == 1
		ret = HAL_SPI_TransmitReceive(ads130b04_context.spi_handle, &tx_buff[0], &rx_buff[0], word_size * len, 500);
#else
        ret = HAL_SPI_Transmit(ads130b04_context.spi_handle, &tx_buff[0], word_size * len, 500);
        ADS130B04_SPI_Deselect();
#endif
    }
    else {
        ret = HAL_ERROR;
    }

#if USE_SPI_TXRX_FUNC == 1
#else
    if (ret == HAL_OK) {
        ADS130B04_SPI_Select();
        ret = HAL_SPI_Receive(ads130b04_context.spi_handle, &rx_buff[0], word_size * len, 500);
    }
#endif

    if (ret == HAL_OK) {
        /* Parse Packet */
        _parseWordPacket(&rx_buff[0], len, p_rx_data);
    }

    ADS130B04_SPI_Deselect();

    return ret;
}

HAL_StatusTypeDef _ads130b04Transmit(uint16_t *p_data, uint8_t len) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t tx_buff[ADS130B04_SPI_WORD_MIN_BUFF_SIZE] = {
        0, };
    uint8_t word_size = ads130b04_context.word_size.byte_num;

    if (ads130b04_context.spi_handle != NULL) {
        /* Make Packet */
        _makeWordPacket(p_data, len, &tx_buff[0]);

        /* Send Data */
        ADS130B04_SPI_Select();
        ret = HAL_SPI_Transmit(ads130b04_context.spi_handle, &tx_buff[0], word_size * len, 500);
    }
    else {
        ret = HAL_ERROR;
    }

    ADS130B04_SPI_Deselect();

    return ret;
}

HAL_StatusTypeDef _ads130b04Receive(int16_t *p_data, uint8_t len) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t rx_buff[ADS130B04_SPI_WORD_MAX_BUFF_SIZE] = {
        0, };
    uint8_t word_size = ads130b04_context.word_size.byte_num;

    if (ads130b04_context.spi_handle != NULL) {
        /* Read Data */
        ADS130B04_SPI_Select();
        ret = HAL_SPI_Receive(ads130b04_context.spi_handle, &rx_buff[0], word_size * len, 500);

        /* Parse Packet */
        _parseWordPacket(&rx_buff[0], len, p_data);
    }
    else {
        ret = HAL_ERROR;
    }

    ADS130B04_SPI_Deselect();

    return ret;
}

void _makeWordPacket(uint16_t *p_data, uint8_t len, uint8_t *p_result) {
    uint8_t word_size = ads130b04_context.word_size.byte_num;

    for (uint8_t idx = 0; idx < len; idx++) {
        p_result[word_size * idx] = (p_data[idx] >> 8) & 0xFF;
        p_result[word_size * idx + 1] = p_data[idx] & 0xFF;

        /* padding.. */
    }
}

void _parseWordPacket(uint8_t *p_data, uint8_t len, int16_t *p_result) {
    uint8_t word_size = ads130b04_context.word_size.byte_num;

    for (uint8_t idx = 0; idx < len; idx++) {
        p_result[idx] = (((p_data[word_size * idx] << 8) & 0xFF00) | (p_data[word_size * idx + 1] & 0xFF));
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ADC_DRDY__Pin) {
        if (ads130b04_context.ads130b04_read == true) {
            _ads130b04GetData(false);
        }
    }
}
