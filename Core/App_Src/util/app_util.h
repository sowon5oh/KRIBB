/**
 ******************************************************************************
 * @file           : app_util.h
 * @brief          : Header file containing common macros used throughout the project.
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
#ifndef _APP_UTIL_H_
#define _APP_UTIL_H_

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define SYS_VERIFY_PARAM_NOT_NULL( param ) \
    do \
    { \
        if( ( param ) == NULL ) \
        { \
            SYS_LOG_ERR( "[Failed] Null Parameter" ); \
            return HAL_ERROR; \
        } \
    } while( 0 );

#define SYS_VERIFY_PARAM_NOT_NULL_VOID( param ) \
    do \
    { \
        if( ( param ) == NULL ) \
        { \
            SYS_LOG_ERR( "[Failed] Null Parameter" ); \
            return; \
        } \
    } while( 0 );

#define SYS_VERIFY_SUCCESS( statement ) \
    do \
    { \
        uint32_t _err_code = ( uint32_t ) ( statement ); \
        if( _err_code != HAL_OK ) \
        { \
            SYS_LOG_ERR( "[Failed] Error code: %d", _err_code ); \
            return _err_code; \
        } \
    } while( 0 );

#define SYS_VERIFY_SUCCESS_VOID( statement ) \
    do \
    { \
        uint32_t _err_code = ( uint32_t ) ( statement ); \
        if( _err_code != HAL_OK ) \
        { \
            SYS_LOG_ERR( "[Failed] Error code: %d", _err_code ); \
            return; \
        } \
    } while( 0 );

#define SYS_VERIFY_VALID_STATE( state ) \
    do \
    { \
        if( !( state ) ) \
        { \
            SYS_LOG_ERR( "[Failed] Invalid State" ); \
            return HAL_ERROR; \
        } \
    } while( 0 );

#define SYS_VERIFY_VALID_STATE_VOID( state ) \
    do \
    { \
        if( !( state ) ) \
        { \
            SYS_LOG_ERR( "[Failed] Invalid State" ); \
            return; \
        } \
    } while( 0 );

#define SYS_VERIFY_TRUE( state ) \
    do \
    { \
        if( !( state ) ) \
        { \
            SYS_LOG_ERR( "[Failed] Invalid" ); \
            return HAL_ERROR; \
        } \
    } while( 0 );

#define SYS_VERIFY_TRUE_VOID( state ) \
    do \
    { \
        if( !( state ) ) \
        { \
            SYS_LOG_ERR( "[Failed] Invalid" ); \
            return; \
        } \
    } while( 0 );

/**
 * @brief Define Bit-field mask
 *
 * Macro that defined the mask with selected number of bits set, starting from
 * provided bit number.
 *
 * @param[in] bcnt Number of bits in the bit-field
 * @param[in] boff Lowest bit number
 */
#define BF_MASK(bcnt, boff) ( ((1U << (bcnt)) - 1U) << (boff) )

/**
 * @brief Get bit-field
 *
 * Macro that extracts selected bit-field from provided value
 *
 * @param[in] val  Value from which selected bit-field would be extracted
 * @param[in] bcnt Number of bits in the bit-field
 * @param[in] boff Lowest bit number
 *
 * @return Value of the selected bits
 */
#define BF_GET(val, bcnt, boff) ( ( (val) & BF_MASK((bcnt), (boff)) ) >> (boff) )

/**
 * @brief Create bit-field value
 *
 * Value is masked and shifted to match given bit-field
 *
 * @param[in] val  Value to set on bit-field
 * @param[in] bcnt Number of bits for bit-field
 * @param[in] boff Offset of bit-field
 *
 * @return Value positioned of given bit-field.
 */
#define BF_VAL(val, bcnt, boff) ( (((uint32_t)(val)) << (boff)) & BF_MASK(bcnt, boff) )

/**
 * @brief Convert a uint16_t value into a 2-byte uint8_t array
 *
 * This macro splits a 16-bit unsigned integer (uint16_t) into two 8-bit values and
 * stores the result in a provided uint8_t array. The most significant byte is stored
 * in the first position of the array, and the least significant byte in the second position.
 *
 * @param[out] arr  A pointer to a 2-element uint8_t array where the result will be stored
 * @param[in]  val  The uint16_t value to be split into two 8-bit values
 */
#define UINT16_TO_UINT8_2BYTE_ARRAY(arr, val) \
    do { \
        (arr)[0] = (uint8_t)((val) >> 8);  /* Store the most significant byte in arr[0] */ \
        (arr)[1] = (uint8_t)((val) & 0xFF); /* Store the least significant byte in arr[1] */ \
    } while (0)

/**
 * @brief Convert a 2-byte uint8_t array into a uint16_t value
 *
 * This macro combines two 8-bit values from a given uint8_t array into a single
 * 16-bit unsigned integer (uint16_t). The first byte is treated as the most significant
 * byte, and the second byte as the least significant byte.
 *
 * @param[in] arr  A pointer to a 2-element uint8_t array containing the two bytes to combine
 *
 * @return The combined 16-bit value as a uint16_t.
 */
#define UINT8_2BYTE_ARRAY_TO_UINT16(arr) \
    ((uint16_t)((arr)[0] << 8) | (uint16_t)(arr)[1])

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _APP_UTIL_H_ */
