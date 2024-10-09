/**
 ******************************************************************************
 * @file           : hal_drv_pd.h
 * @brief          : Header file for hal_drv_pd.c.
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
#ifndef _HAL_DRV_PD_H
#define _HAL_DRV_PD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
typedef enum {
    HAL_PD_CAT_RECV = 0,
    HAL_PD_CAT_MONITOR,
    HAL_PD_CAT_MAX = HAL_PD_CAT_MONITOR,
} HalPdCat_t;

typedef enum {
    HAL_PD_CH_NUM_1 = 0,
    HAL_PD_CH_NUM_2,
    HAL_PD_CH_NUM_3,
    HAL_PD_CH_NUM_NUM,
} HalPdCh_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Hal_Pd_Init(SPI_HandleTypeDef *p_hdl);
HAL_StatusTypeDef Hal_Pd_Start(void);
HAL_StatusTypeDef Hal_Pd_Stop(void);
HAL_StatusTypeDef Hal_Pd_GetRecvData(HalPdCh_t ch, uint16_t *p_data);
HAL_StatusTypeDef Hal_Pd_GetMonitorData(HalPdCh_t ch, uint16_t *p_data);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DRV_PD_H */
