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

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _APP_UTIL_H_ */
