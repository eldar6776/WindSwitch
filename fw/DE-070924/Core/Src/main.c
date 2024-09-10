/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
ADC_HandleTypeDef hadc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
#define IDLE   0
#define DONE   1
#define F_CLK  8000000UL
#define ADC_READOUT_PERIOD 100
#define TRIGGER_TIME 2000 // 1s wind treshold to trigger shutter down

volatile uint8_t gu8_State = IDLE, tick = 0, tickcnt = 0, redled = 0, init = 0;
volatile uint8_t gu8_MSG[35] = {'\0'};
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile uint16_t gu16_TIM3_OVC = 0;
volatile uint32_t gu32_Freq = 0;
volatile uint32_t ledtmr, ledtime = 200, nulltmr = 0, trgtmr = 0;
volatile uint32_t redledtmr = 0;
volatile uint8_t setpoint = 0, delay = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void LED(void);
static void OUT(void);
static void ADC3_Read(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
	MX_ADC_Init();
	MX_IWDG_Init();
	MX_TIM3_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while(1)
	{
		ADC3_Read();
		LED();
		OUT();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if(nulltmr && (HAL_GetTick() - nulltmr >= 2000))
		{
			gu32_Freq = 0;
			nulltmr = 0;
		}

#ifdef USE_WATCHDOG
		HAL_IWDG_Refresh(&hiwdg);
#endif
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14
	                                   | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                              | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = DISABLE;//ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = ENABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if(HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	*/
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if(HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	*/
	sConfig.Channel = ADC_CHANNEL_1;
	if(HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */
#ifdef USE_WATCHDOG
	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */
#endif
	/* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if(HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_TIM_IC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if(HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, OUTCTRL_Pin | STATUS_LED_Pin | HOLD_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : OUTCTRL_Pin STATUS_LED_Pin HOLD_LED_Pin */
	GPIO_InitStruct.Pin = OUTCTRL_Pin | STATUS_LED_Pin | HOLD_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if(gu8_State == IDLE)
	{
		gu32_T1 = TIM3->CCR4;
		gu16_TIM3_OVC = 0;
		gu8_State = DONE;
	}
	else if(gu8_State == DONE)
	{
		gu32_T2 = TIM3->CCR4;
		gu32_Ticks = (gu32_T2 + (gu16_TIM3_OVC * 65536)) - gu32_T1;
		gu32_Freq = (uint32_t)(F_CLK / gu32_Ticks) + 1;
		gu8_State = IDLE;
		tick = 1;
	}
	nulltmr = HAL_GetTick();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	gu16_TIM3_OVC++;
}

static void OUT(void)
{
	if(!init) return;

	switch(redled)
	{
		case 0:
			if(setpoint < gu32_Freq)
			{
				trgtmr = HAL_GetTick();
				++redled;
			}
			break;
		case 1:
			if(setpoint >= gu32_Freq)
			{
				redled = 0;
			}
			else if((HAL_GetTick() - trgtmr) >= TRIGGER_TIME)
			{
				HAL_GPIO_WritePin(OUTCTRL_GPIO_Port, OUTCTRL_Pin, GPIO_PIN_SET);
				trgtmr = HAL_GetTick();
				++redled;
			}
			break;
		case 2:
			if(setpoint < gu32_Freq)
			{
				trgtmr = HAL_GetTick();
			}
			else if((HAL_GetTick() - trgtmr) >= (delay * 60000U))
			{
				HAL_GPIO_WritePin(OUTCTRL_GPIO_Port, OUTCTRL_Pin, GPIO_PIN_RESET);
				trgtmr = 0;
				redled = 0;
				redledtmr = 0;
			}
			break;
		default:
			trgtmr = 0;
			redled = 0;
			redledtmr = 0;
			break;

	}
}

static void LED(void)
{
	if(tick)
	{
		if(tickcnt == 0)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			ledtmr = HAL_GetTick();
			++tickcnt;
		}
		else if(tickcnt == 1)
		{
			if((HAL_GetTick() - ledtmr) >= 5)
			{
				tick = 0;
				tickcnt = 0;
				ledtmr = HAL_GetTick();
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			}
		}
	}
	else if((HAL_GetTick() - ledtmr) >= ledtime)
	{
		ledtmr = HAL_GetTick();
		if(HAL_GPIO_ReadPin(STATUS_LED_GPIO_Port, STATUS_LED_Pin) == GPIO_PIN_SET)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			ledtime = 5000;
		}
		else
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			ledtime = 5;
		}
	}

	if(redled == 0)
	{
		HAL_GPIO_WritePin(HOLD_LED_GPIO_Port, HOLD_LED_Pin, GPIO_PIN_RESET);
	}
	else if(redled == 1)
	{
		if((HAL_GetTick() - redledtmr) >= 100)
		{
			redledtmr = HAL_GetTick();
			HAL_GPIO_TogglePin(HOLD_LED_GPIO_Port, HOLD_LED_Pin);
		}
	}
	else if(redled == 2) HAL_GPIO_WritePin(HOLD_LED_GPIO_Port, HOLD_LED_Pin, GPIO_PIN_SET);

}
/**
  * @brief
  * @param
  * @retval
  */
static void ADC3_Read(void)
{
	ADC_ChannelConfTypeDef sConfig;

	static uint32_t adc_timer = 0U;
	static uint32_t setpoint_sample_cnt = 0U;
	static uint32_t delay_sample_cnt = 0U;
	static uint16_t setpoint_sample[10] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
	static uint16_t delay_sample[10] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
	static uint8_t adc_cnt = 0U;
	uint32_t tmp_val = 0, t = 0;


	if((HAL_GetTick() - adc_timer) >= ADC_READOUT_PERIOD)
	{
		adc_timer = HAL_GetTick();
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
		if(adc_cnt == 0U) sConfig.Channel = ADC_CHANNEL_0;
		else if(adc_cnt == 1U) sConfig.Channel = ADC_CHANNEL_1;
		HAL_ADC_ConfigChannel(&hadc, &sConfig);
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 10);

		if(adc_cnt == 0U)
		{
			setpoint_sample[setpoint_sample_cnt] = HAL_ADC_GetValue(&hadc);
			if(++setpoint_sample_cnt >  9U) setpoint_sample_cnt = 0U;
			tmp_val = 0U;
			for(t = 0U; t < 10U; t++) tmp_val += setpoint_sample[t];
			tmp_val = tmp_val / 10U;
			setpoint = map(tmp_val, 0, 4095, 0, 100);
			++adc_cnt;
		}
		else if(adc_cnt == 1U)
		{
			delay_sample[delay_sample_cnt] = HAL_ADC_GetValue(&hadc);
			if(++delay_sample_cnt >  9U)
			{
				delay_sample_cnt = 0U;
				init = 1;
			}
			tmp_val = 0U;
			for(t = 0U; t < 10U; t++) tmp_val += delay_sample[t];
			tmp_val = tmp_val / 10U;
			delay = map(tmp_val, 0, 4095, 0, 100);
			adc_cnt = 0;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while(1)
	{
		HAL_NVIC_SystemReset();
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
