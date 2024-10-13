/**
 ******************************************************************************
 * @file           : util_debug.h
 * @brief          : Header file for util_debug.c.
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
#ifndef _UTIL_DEBUG_H_
#define _UTIL_DEBUG_H_

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define DBG_METADATA_ARGS __FUNCTION__, __LINE__

#define SYS_LOG_LEVEL_NONE    0
#define SYS_LOG_LEVEL_DEBUG   1
#define SYS_LOG_LEVEL_INFO    2
#define SYS_LOG_LEVEL_WARN    3
#define SYS_LOG_LEVEL_ERR     4

#if (SYS_LOG_ENABLE == 1)
#define USER_SYS_LOG_LEVEL    SYS_LOG_LEVEL_DEBUG
#else
#define USER_SYS_LOG_LEVEL    SYS_LOG_LEVEL_NONE
#endif

/* Exported macro ------------------------------------------------------------*/
void DEBUG_Printf(const char *format, ...);
#define PRINTFUNCTION(format, args...)    DEBUG_Printf(format, ## args)

#if (USER_SYS_LOG_LEVEL == SYS_LOG_LEVEL_NONE)
#define SYS_LOG_TEST( message, args ... )
#define SYS_LOG_DEBUG( message, args ... )
#define SYS_LOG_INFO( message, args ... )
#define SYS_LOG_WARN( message, args ... )
#define SYS_LOG_ERR( message, args ... )
#elif (USER_SYS_LOG_LEVEL == SYS_LOG_LEVEL_DEBUG)
#define SYS_LOG_TEST( message, args ... )   PRINTFUNCTION( "[TEST] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define SYS_LOG_DEBUG( message, args ... )   PRINTFUNCTION( "[DEBUG] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define SYS_LOG_INFO( message, args ... )    PRINTFUNCTION( "[INFO] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define SYS_LOG_WARN( message, args ... )    PRINTFUNCTION( "[WARN] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define SYS_LOG_ERR( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#elif (USER_SYS_LOG_LEVEL == SYS_LOG_LEVEL_INFO)
#define SYS_LOG_TEST( message, args ... )
#define SYS_LOG_DEBUG( message, args ... )
#define SYS_LOG_INFO( message, args ... )    PRINTFUNCTION( "[INFO] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define SYS_LOG_WARN( message, args ... )    PRINTFUNCTION( "[WARN] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define SYS_LOG_ERR( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#elif (USER_SYS_LOG_LEVEL == SYS_LOG_LEVEL_WARN)
#define SYS_LOG_TEST( message, args ... )
#define SYS_LOG_DEBUG( message, args ... )
#define SYS_LOG_INFO( message, args ... )
#define SYS_LOG_WARN( message, args ... )    PRINTFUNCTION( "[WARN] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define SYS_LOG_ERR( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#elif (USER_SYS_LOG_LEVEL == SYS_LOG_LEVEL_ERR)
#define SYS_LOG_TEST( message, args ... )
#define SYS_LOG_DEBUG( message, args ... )
#define SYS_LOG_INFO( message, args ... )
#define SYS_LOG_WARN( message, args ... )
#define SYS_LOG_ERR( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#endif

/* Exported functions prototypes ---------------------------------------------*/

#endif /* _UTIL_DEBUG_H_ */
