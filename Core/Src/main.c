/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : main program body
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

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if(CONFIG_FEATURE_TEMPERATURE_DMA_MODE != 1)
#include "hal_drv_temperature.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint16_t led_timer_1sec_cnt;
static AppTimer_t app_timers[APP_TIMER_ID_MAX];
static AppTask_t app_tasks[APP_TASK_ID_MAX];

#define	UART_DEBUG_MSG 	1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch) {
    if (HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 10) != HAL_OK)
        return -1;
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Timer interrupt callback function.
 *
 * This function is called when the timer reaches the end of its period.
 * It is designed to handle a 1 kHz timer (1 ms tick) and increment a counter for a 1-second LED timer.
 * Additionally, it checks the application timers (handled by TIM7) and decrements their remaining time.
 * When an application timer reaches 0, it triggers a user-defined callback function if it's set.
 *
 * - 1 ms tick increments the LED 1-second counter.
 * - Monitors the application timers in TIM7, decrements their remaining time,
 *   and calls the registered callback when the timer expires.
 *
 * @param htim  TIM handle associated with the timer interrupt.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* 1 kHz APP Timer (1 msec) */
    if (htim->Instance == TIM7) {
        /* 1 kHz LED/FSM timer (1 sec) */
        if (++led_timer_1sec_cnt >= 1000) {
            HAL_GPIO_TogglePin(OP_LED_GPIO_Port, OP_LED_Pin);
            led_timer_1sec_cnt = 0;
        }
    }

    /* 10 kHz Timer (0.1 msec) */
    if (htim->Instance == TIM10) {
        /* app timer */
        for (uint8_t timer_idx = 0; timer_idx < APP_TIMER_ID_MAX; timer_idx++) {
            /* Check if the timer is active and if there is remaining time */
            if (app_timers[timer_idx].active && app_timers[timer_idx].remaining_0_1_ms > 0) {
                app_timers[timer_idx].remaining_0_1_ms--; /* Decrease the remaining time by 1 ms */
                if (app_timers[timer_idx].remaining_0_1_ms == 0) {
                    if (NULL != app_timers[timer_idx].timer_cb) {
                        app_timers[timer_idx].timer_cb(); /* Call the timer callback function */
                        if (app_timers[timer_idx].repeat) {
                            app_timers[timer_idx].remaining_0_1_ms = app_timers[timer_idx].timeout_0_1_ms; /* Restart Timer */
                        }
                        else {
                            app_timers[timer_idx].active = 0; /* Stop Timer */
                        }
                    }
                }
            }
        }

        for (uint8_t task_idx = 0; task_idx < APP_TASK_ID_MAX; task_idx++) {
            /* Check if the task is active and if there is remaining time */
            if (app_tasks[task_idx].active && app_tasks[task_idx].remaining_0_1_ms > 0) {
                app_tasks[task_idx].remaining_0_1_ms--; /* Decrease the remaining time by 1 ms */
                if (app_tasks[task_idx].remaining_0_1_ms == 0) {
                    if (NULL != app_tasks[task_idx].task_cb) {
                        app_tasks[task_idx].task_cb(); /* Call the task callback function */
                        app_tasks[task_idx].remaining_0_1_ms = app_tasks[task_idx].task_duty_0_1_ms; /* task keep going */
                    }
                }
            }
        }
    }
}

#if(CONFIG_FEATURE_TEMPERATURE_DMA_MODE != 1)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        Hal_Temp_AdcCb();
    }
}
#endif

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    /* USER CODE BEGIN 2 */
#if (UART_DEBUG_MSG == 1)
    setbuf(stdout, NULL);
#endif

    /* Timer Start */
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim10);

    /* User Module Initialize */
    (void) UART_Init(&huart1);

#define TEST_UART_TX 0
#if (TEST_UART_TX == 1)
    char Tx_str_data[] = "UNIOTECH CORP.\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*) Tx_str_data, sizeof(Tx_str_data) - 1, 1000);
#endif
    /***********************************************************************
     * System Init
     ***********************************************************************/
    // @formatter:off
    SYS_LOG_INFO("----------------------------------");
    SYS_LOG_INFO("  [UNIOTECH] 3CH FL ANALYZER UOT  ");
    SYS_LOG_INFO("* Build Time: %s %s", __DATE__, __TIME__);
    SYS_LOG_INFO("* FW Ver    : Ver. %d.%d.%02d", SYS_FW_MAJOR_VER, SYS_FW_MINOR_VER, SYS_FW_PATCH_VER);
    SYS_LOG_INFO("* HW Ver    : Ver. %d.%d", SYS_HW_MAJOR_VER, SYS_HW_MINOR_VER);
    SYS_LOG_INFO("----------------------------------");

    SYS_LOG_INFO("----------------------------------");
    SYS_LOG_INFO("            Devie Start           ");
    SYS_LOG_INFO("----------------------------------");
                                                                                                                        // @formatter:on
    /* Init Fsm Task */
    Task_Fsm_Init();
    
    /* Init Meas Task & Read settings from FRAM */
    HAL_Delay(10);
    Task_Meas_Init();
    Task_TempCtrl_Init();
    
#if (SYS_TEST_MODE_ENABLE != 1)
    /***********************************************************************
     * Test Sequence
     ***********************************************************************/
    HAL_Delay(1000);
    Task_TempCtrl_Start();

    HAL_Delay(1000);
    Task_Meas_Start();
#else
    /***********************************************************************
     * Test Sequence
     ***********************************************************************/
    /* Add Test Sequence for user tests */
//    Task_Fsm_StartTest(FSM_TEST_DEVICE_HEATER_ONOFF, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_DEVICE_READ_TEMP, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_SET_TEMP_ONOFF, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_SET_LED_ON_TIME, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_SET_LED_ON_LEVEL, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_SET_ADC_SAMPLE_CNT, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_SET_ADC_DELAY_MS, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_SET_STABLE_TEMPERATURE, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_SET_REQ_ALL_SETTINGS, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_REQ_TEMPERATURE, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_REQ_RECV_PD, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_REQ_MONITOR_PD, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_REQ_MEASURE, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_DEV_CTRL_LED_ONOFF, FSM_TEST_MODE_SEQUENCE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_DEV_STATUS_REQ, FSM_TEST_MODE_SINGLE);
//    Task_Fsm_StartTest(FSM_TEST_MMI_REQ_MEASURE, FSM_TEST_MODE_SINGLE);
#endif
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {
        0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
        0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {
        0 };
    ADC_InjectionConfTypeDef sConfigInjected = {
        0 };

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 3;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
    sConfigInjected.InjectedRank = 1;
    sConfigInjected.InjectedNbrOfConversion = 3;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }

    /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedRank = 2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }

    /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedRank = 3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {
        0 };

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 90 - 1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 1000 - 1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 179;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 99;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */

    /* USER CODE END TIM10_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 460800;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {
        0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, CH1_LED_CON_Pin | CH2_LED_CON_Pin | CH3_LED_CON_Pin | ADC_RST__Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, M_SEL_A0_Pin | M_SEL_A1_Pin | M_SEL_EN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ADC_CS__GPIO_Port, ADC_CS__Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, OP_LED_Pin | EEPROM_WP_Pin | HEAT_CON3_Pin | HEAT_CON2_Pin | HEAT_CON1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : CH1_LED_CON_Pin CH2_LED_CON_Pin CH3_LED_CON_Pin ADC_RST__Pin */
    GPIO_InitStruct.Pin = CH1_LED_CON_Pin | CH2_LED_CON_Pin | CH3_LED_CON_Pin | ADC_RST__Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : M_SEL_A0_Pin M_SEL_A1_Pin M_SEL_EN_Pin */
    GPIO_InitStruct.Pin = M_SEL_A0_Pin | M_SEL_A1_Pin | M_SEL_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ADC_CS__Pin */
    GPIO_InitStruct.Pin = ADC_CS__Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADC_CS__GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : OP_LED_Pin EEPROM_WP_Pin HEAT_CON3_Pin HEAT_CON2_Pin
     HEAT_CON1_Pin */
    GPIO_InitStruct.Pin = OP_LED_Pin | EEPROM_WP_Pin | HEAT_CON3_Pin | HEAT_CON2_Pin | HEAT_CON1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : ADC_DRDY__Pin */
    GPIO_InitStruct.Pin = ADC_DRDY__Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ADC_DRDY__GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void App_Timer_Start(AppTimerId_t timer_id, uint32_t timeout_ms, bool repeat, void (*timer_cb)(void)) {
    if (timer_id < APP_TIMER_ID_MAX) {
        app_timers[timer_id].timeout_0_1_ms = timeout_ms * 10;
        app_timers[timer_id].remaining_0_1_ms = timeout_ms * 10;
        app_timers[timer_id].timer_cb = timer_cb;
        app_timers[timer_id].active = 1;
        app_timers[timer_id].repeat = repeat;
    }
}

void App_Timer_Stop(AppTimerId_t timer_id) {
    if (timer_id < APP_TIMER_ID_MAX) {
        app_timers[timer_id].active = 0;
        app_timers[timer_id].repeat = false;
    }
}

void App_Task_Start(AppTaskId_t task_id, uint32_t task_duty_ms, void (*task_cb)(void)) {
    if (task_id < APP_TASK_ID_MAX) {
        app_tasks[task_id].task_duty_0_1_ms = task_duty_ms * 10;
        app_tasks[task_id].remaining_0_1_ms = task_duty_ms * 10;
        app_tasks[task_id].task_cb = task_cb;
        app_tasks[task_id].active = 1;
    }
}

void App_Task_Stop(AppTaskId_t task_id) {
    if (task_id < APP_TASK_ID_MAX) {
        app_tasks[task_id].active = 0;
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
