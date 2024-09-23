/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _UTIL_DEBUG_H_
#define _UTIL_DEBUG_H_

void DEBUG_Printf(const char *format, ...);
#define PRINTFUNCTION(format, args...)    DEBUG_Printf(format, ## args)

#define DBG_METADATA_ARGS      __FUNCTION__, __LINE__

#define LOG_LEVEL_NONE   0
#define LOG_LEVEL_DEBUG  1
#define LOG_LEVEL_INFO   2
#define LOG_LEVEL_WARN   3
#define LOG_LEVEL_ERROR  4

#define USER_LOG_LEVEL   LOG_LEVEL_DEBUG

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

#endif /* _UTIL_DEBUG_H_ */
