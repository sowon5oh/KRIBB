/**
 ******************************************************************************
 * @file           : drv_tempsensor_lmt86lp.c
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
#include "drv_tempsensor_lmt86lp.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#ifdef FEATURE_TEMPERATURE_DATA_TYPE
#if (FEATURE_TEMPERATURE_DATA_DEGREE == FEATURE_TEMPERATURE_DATA_TYPE)
static float _interpolate(uint32_t vout, uint32_t vout1, float temp1, uint32_t vout2, float temp2);
static float _temp_converter_from_vout(uint32_t vout);
#endif
#endif

/* Private variables ---------------------------------------------------------*/
#ifdef FEATURE_TEMPERATURE_DATA_TYPE
#if (FEATURE_TEMPERATURE_DATA_DEGREE == FEATURE_TEMPERATURE_DATA_TYPE)
static uint32_t lmt86lp_vout_table[] = { 2616, 2607, 2598, 2589, 2580, 2571, 2562, 2553, 2543, 2533, 2522, 2512, 2501, 2491, 2481, 2470, 2460, 2449, 2439, 2429, 2418, 2408, 2397,
    2387, 2376, 2366, 2355, 2345, 2334, 2324, 2313, 2302, 2292, 2281, 2271, 2260, 2250, 2239, 2228, 2218, 2207, 2197, 2186, 2175, 2164, 2154, 2143, 2132, 2122, 2111, 2100, 2089,
    2079, 2068, 2057, 2047, 2036, 2025, 2014, 2004, 1993, 1982, 1971, 1961, 1950, 1939, 1928, 1918, 1907, 1896, 1885, 1874, 1864, 1853, 1842, 1831, 1820, 1810, 1799, 1788, 1777,
    1766, 1756, 1745, 1734, 1723, 1712, 1701, 1690, 1679, 1668, 1657, 1646, 1635, 1624, 1613, 1602, 1591, 1580, 1569, 1558, 1547, 1536, 1525, 1514, 1503, 1492, 1481, 1470, 1459,
    1448, 1436, 1425, 1414, 1403, 1391, 1380, 1369, 1358, 1346, 1335, 1324, 1313, 1301, 1290, 1279, 1268, 1257, 1245, 1234, 1223, 1212, 1201, 1189, 1178, 1167, 1155, 1144, 1133,
    1122, 1110, 1099, 1088, 1076, 1065, 1054, 1042, 1031, 1020, 1008, 997, 986, 974, 963, 951, 940, 929, 917, 906, 895, 883, 872, 860, 849, 837, 826, 814, 803, 791, 780, 769, 757,
    745, 734, 722, 711, 699, 688, 676, 665, 653, 642, 630, 618, 607, 595, 584, 572, 560, 549, 537, 525, 514, 502, 490, 479, 467, 455, 443, 432, 420 };
static float lmt86lp_temp_table[] = { -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23,
    -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
    65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106,
    107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141,
    142, 143, 144, 145, 146, 147, 148, 149, 150 };
#define LMT86LP_TABLE_SIZE 201
#endif
#endif

ADC_HandleTypeDef *tempsensor_adc_hdl;

/* Public user code ----------------------------------------------------------*/
HAL_StatusTypeDef DRV_LMT86LP_Init(ADC_HandleTypeDef *p_hdl) {
    /* ADC handle registration */
    SYS_VERIFY_PARAM_NOT_NULL(p_hdl);
    tempsensor_adc_hdl = p_hdl;

    return HAL_OK;
}

HAL_StatusTypeDef DRV_LMT86LP_Start(void) {
    return HAL_ADC_Start_IT(tempsensor_adc_hdl);
}

HAL_StatusTypeDef DRV_LMT86LP_Stop(void) {
    return HAL_ADC_Stop_IT(tempsensor_adc_hdl);
}

HAL_StatusTypeDef DRV_LMT86LP_GetValue(HalTempData_t *p_data) {
    uint16_t adc_val[3];

    adc_val[0] = (uint16_t) HAL_ADC_GetValue(tempsensor_adc_hdl);
    adc_val[1] = (uint16_t) HAL_ADC_GetValue(tempsensor_adc_hdl);
    adc_val[2] = (uint16_t) HAL_ADC_GetValue(tempsensor_adc_hdl);

#ifdef FEATURE_TEMPERATURE_DATA_TYPE
#if (FEATURE_TEMPERATURE_DATA_ADC == FEATURE_TEMPERATURE_DATA_TYPE)
    p_data->ch1_temp = adc_val[0];
    p_data->ch2_temp = adc_val[1];
    p_data->ch3_temp = adc_val[2];
#else
    p_data->ch1_temp = _temp_converter_from_vout(adc_val[0]);
        p_data->ch2_temp = _temp_converter_from_vout(adc_val[1]);
        p_data->ch3_temp = _temp_converter_from_vout(adc_val[2]);
#endif
#endif
    return HAL_OK;
}

/* Private user code ---------------------------------------------------------*/
#ifdef FEATURE_TEMPERATURE_DATA_TYPE
#if (FEATURE_TEMPERATURE_DATA_DEGREE == FEATURE_TEMPERATURE_DATA_TYPE)
// Linear interpolation function
static float _interpolate(uint32_t vout, uint32_t vout1, float temp1, uint32_t vout2, float temp2) {
    return temp1 + ((vout - vout1) * (temp2 - temp1) / (vout2 - vout1));
}

// Function to get temperature from Vout using lookup table
static float _temp_converter_from_vout(uint32_t vout) {
    // If Vout is below the range, return the lowest temperature
    if (vout <= lmt86lp_vout_table[0]) {
        return lmt86lp_temp_table[0];
    }

    // If Vout is above the range, return the highest temperature
    if (vout >= lmt86lp_vout_table[LMT86LP_TABLE_SIZE - 1]) {
        return lmt86lp_temp_table[LMT86LP_TABLE_SIZE - 1];
    }

    // Search in the lookup table
    for (uint8_t idx = 0; idx < LMT86LP_TABLE_SIZE - 1; idx++) {
        if (vout >= lmt86lp_vout_table[idx] && vout <= lmt86lp_vout_table[idx + 1]) {
            // Linear interpolation between two points in the table
            return _interpolate(vout, lmt86lp_vout_table[idx], lmt86lp_temp_table[idx], lmt86lp_vout_table[idx + 1], lmt86lp_temp_table[idx + 1]);
        }
    }

    // Default case (should not occur)
    return -1.0f;
}
#endif
#endif
