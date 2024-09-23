/**
 ******************************************************************************
 * @file           : xx.c
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
#include <stdarg.h>
#include "util_debug.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static char msg[512] = { '\0', };

/* Public user code ----------------------------------------------------------*/
void DEBUG_Printf(const char *format, ...) {
	va_list args;
	memset(msg, 0, sizeof(msg));

	va_start(args, format);
	vsnprintf(msg, sizeof(msg), format, args);
	va_end(args);

	printf("%s\r\n", msg);
}

/* Private user code ---------------------------------------------------------*/

