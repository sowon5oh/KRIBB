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

#define LOG_LEVEL_NONE    0
#define LOG_LEVEL_DEBUG   1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_WARN    3
#define LOG_LEVEL_ERROR   4

#define USER_LOG_LEVEL    LOG_LEVEL_DEBUG

/* Exported macro ------------------------------------------------------------*/
void DEBUG_Printf(const char *format, ...);
#define PRINTFUNCTION(format, args...)    DEBUG_Printf(format, ## args)

#if (USER_LOG_LEVEL == LOG_LEVEL_NONE)
#define LogDebug( message, args ... )
#define LogInfo( message, args ... )
#define LogWarn( message, args ... )
#define LogError( message, args ... )
#elif (USER_LOG_LEVEL == LOG_LEVEL_DEBUG)
#define LogDebug( message, args ... )   PRINTFUNCTION( "[DEBUG] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define LogInfo( message, args ... )    PRINTFUNCTION( "[INFO] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define LogWarn( message, args ... )    PRINTFUNCTION( "[WARN] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define LogError( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#elif (USER_LOG_LEVEL == LOG_LEVEL_INFO)
#define LogDebug( message, args ... )
#define LogInfo( message, args ... )    PRINTFUNCTION( "[INFO] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define LogWarn( message, args ... )    PRINTFUNCTION( "[WARN] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define LogError( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#elif (USER_LOG_LEVEL == LOG_LEVEL_WARN)
#define LogDebug( message, args ... )
#define LogInfo( message, args ... )
#define LogWarn( message, args ... )    PRINTFUNCTION( "[WARN] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#define LogError( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#elif (USER_LOG_LEVEL == LOG_LEVEL_ERROR)
#define LogDebug( message, args ... )
#define LogInfo( message, args ... )
#define LogWarn( message, args ... )
#define LogError( message, args ... )   PRINTFUNCTION( "[ERROR] [%s: %d] " message, DBG_METADATA_ARGS, ## args )
#endif

/* Exported functions prototypes ---------------------------------------------*/

#endif /* _UTIL_DEBUG_H_ */
