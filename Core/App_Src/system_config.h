/**
 ******************************************************************************
 * @file           : system_config.h
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
#ifndef _SYSTEM_CONFIG_H_
#define _SYSTEM_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* system info */
#define SYS_FW_MAJOR_VER 1
#define SYS_FW_MINOR_VER 0
#define SYS_FW_PATCH_VER 0
#define SYS_HW_MAJOR_VER 2
#define SYS_HW_MINOR_VER 0
#define MODEL_NAME       "KRIBB_FLURESCENCE_ANALYZER"

/* config for test mode */
#define SYS_TEST_MODE_ENABLE 0

/* config for system log*/
#define SYS_LOG_ENABLE_CFG  1

#define SYS_LOG_LEVEL_NONE  0
#define SYS_LOG_LEVEL_DEBUG 1
#define SYS_LOG_LEVEL_INFO  2
#define SYS_LOG_LEVEL_WARN  3
#define SYS_LOG_LEVEL_ERR   4
#define SYS_LOG_LEVEL_CFG   SYS_LOG_LEVEL_INFO

/* config for features */
#define FEATURE_SETTINGS_DEFAULT          0

#define FEATURE_TEMPERATURE_DMA_MODE      1
#define FEATURE_TEMPERATURE_DATA_ADC      0
#define FEATURE_TEMPERATURE_DATA_DEGREE   1
#define FEATURE_TEMPERATURE_DATA_TYPE     FEATURE_TEMPERATURE_DATA_DEGREE
#define FEATURE_TEMPERATURE_DEGREE_OFFSET 30

#define FEATURE_TEST_REQ_FAKE_DATA_ENABLE 0

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _SYSTEM_CONFIG_H_ */
