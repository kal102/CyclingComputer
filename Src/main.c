
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "ride.h"
#include "GPS/UART.h"
#include "GPS/NMEA.h"
#include "GPS/UBX.h"
#include "BMP280/bmp280_user.h"
#include "OLED/OLED_GUI.h"
#include "SD/sd_stm32.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t tick = 0;
volatile uint8_t scheduler = 1;
volatile uint8_t stopFlag = 0;
volatile uint8_t ignoreFirstPress = 0;
volatile uint16_t idleCounter = 0;
volatile uint16_t batteryVoltage = 300;
volatile uint8_t autopausedFlag = 0;
volatile NOTIFICATION notification;

GPS_Data gpsData = {.latitude = 0.0, .latitudeHemisphere = '?', .longitude = 0.0, .longitudeHemisphere = '?', .velocity = 0.0, .course = 0.0, .navigationType = NA,
	.satellitesCount = 0, .hdop = NOSIGNAL, .differentialGPSDataAge = 0, .differentialReferenceStationID = 0, .date = 0, .utcTime = 0.0};
GPS_Data lastGpsData;

char msg[MAX_MSG_LEN];
char currentGPXPath[30];

struct bmp280_data barometerData;
struct bmp280_data lastBarometerData;

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm timeSet;
int8_t timezone = 0;
int8_t oldTimezone = 0;

FATFS fileSystem;
FIL file;
FRESULT fileResult;
char ** fileList = NULL;
uint8_t fileNumber;

const uint8_t BIKE_COUNT = 6;
const char* const bikeTypeArray[] = {"ROAD","TT","GRAVEL","CROSS","MOUNTAIN","FAT"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM10_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void EnterSTOPMode(void);
static void LeaveSTOPMode(void);
static MENU StateMachine(MENU menu, BUTTONS *keys, BUTTONS *oldKeys, uint8_t menuScreens[]);
static BATTERY ReadBatteryState(void);
static BUTTONS ReadButtons(BUTTONS keys, BUTTONS oldKeys);
static uint8_t ReadButtonHoldCount(BUTTON key, BUTTON oldKey);
static void Display(BATTERY battery, MENU menu);
static FRESULT WriteHeaderToSD(void);
static FRESULT WriteTrackpointToSD(void);
static FRESULT WriteEndingToSD(void);
static FRESULT ScanDirectoryOnSD(char* path);
static FRESULT RemoveFileFromSD(char* path);
static void ClearFileList(void);

static inline uint8_t KeyPressed(BUTTON key, BUTTON oldKey);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback()
{
	tick = (tick + 1) % TICKS_PER_SEC;
	scheduler = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM10)
	{
		card_timerproc();
		if (notification.duration > 0)
		{
			if (notification.duration < INFINITE)
			{
				notification.duration--;
			}
		}
		else
		{
			notification.type = NONE;
		}
	}
	if (htim->Instance == TIM11)
	{
		idleCounter++;
		if (Ride_GetState() == STARTED)
		{
			Ride_TimerPulse();
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t value;

	value = HAL_ADC_GetValue(hadc);
	batteryVoltage = (2UL * value * SUPPLY_VOLTAGE) / ADC_MAX_VALUE;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)
	{
		if (stopFlag)
		{
			LeaveSTOPMode();
			notification.type = WELCOME;
			notification.duration = 3000;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  BATTERY battery;
  BUTTONS keys;
  BUTTONS oldKeys;
  MENU menu;
  uint8_t menuScreens[] = {5, 3, 5, 2, 1, 1, 1, 15, 2, 2};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  oldKeys.ok.state = 0;
  oldKeys.cancel.state = 0;
  oldKeys.left.state = 0;
  oldKeys.right.state = 0;

  menu.menu = 0;
  menu.screen = 0;

  lastGpsData = gpsData;
  lastBarometerData = barometerData;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemCoreClock = 32000000UL;
  SysTick_Config(SystemCoreClock / TICKS_PER_SEC);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  MX_TIM10_Init();
  MX_RTC_Init();
  MX_ADC_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DBGMCU_EnableDBGStopMode();
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  LL_SPI_Enable(SPI3);

  System_Init();
  OLED_Init(SCAN_DIR_DFT );
  notification.type = WELCOME;
  notification.duration = 2000;
  Display(battery, menu);

  user_bmp280_init();
  user_bmp280_config();

  Ride_ClearInfo();
  UART_startReceiving();
  UBX_powerSavingOFF();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (scheduler)
	  {
		  scheduler = 0;
		  HAL_GPIO_WritePin(CPU_LOAD_GPIO_Port, CPU_LOAD_Pin, GPIO_PIN_SET);

		  /******************/
		  /* Maszyna stanów */
		  /******************/
		  keys = ReadButtons(keys, oldKeys);
		  menu = StateMachine(menu, &keys, &oldKeys, menuScreens);
		  oldKeys = keys;

		  /*********************/
		  /* Procesy cykliczne */
		  /*********************/
		  if (tick == 10)
		  {
			  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		  }
		  else if (tick == 30)
		  {
			  HAL_ADC_Start_IT(&hadc);
		  }
		  else if (tick == 50)
		  {
			  battery = ReadBatteryState();
		  }
		  else if (tick == 70)
		  {
			  if ((gpsData.hdop >= PRECISION_THRESHOLD) && (lastGpsData.hdop >= PRECISION_THRESHOLD))
			  {
				  if (Ride_GetState() == STOPPED)
				  {
						calculate_pressure_atmospheric(barometerData.temperature, barometerData.pressure, \
								(int32_t)(100 * gpsData.altitude));
				  }
				  Ride_CalculateInfo(&gpsData, &lastGpsData, &barometerData, &lastBarometerData);
				  if (fileResult == FR_OK)
				  {
					  fileResult = WriteTrackpointToSD();
					  if (fileResult != FR_OK)
					  {
						  notification.type = SD_CARD_ERROR;
						  notification.duration = 3000;
					  }
				  }
			  }
			  lastGpsData = gpsData;
			  lastBarometerData = barometerData;
		  }

		  /****************************/
		  /* Procesy dzia³aj¹ce w tle */
		  /****************************/

		  /* Odczytywanie komunikatów GPSa */
		  if (UART_getMsgCounter())
		  {
			  UART_getMessage(msg);
			  NMEA_parseString(msg, &gpsData);
		  }

		  /* Odczytywanie wysokoœci z BMP280 */
		  if ((tick % MEASUREMENT_PERIOD) == 0)
		  {
			  barometerData = user_bmp280_read_average();
		  }

		  /* Odœwie¿anie wyœwietlacza */
		  if ((tick % DISPLAY_PERIOD) == 0)
		  {
			  Display(battery, menu);
		  }

		  HAL_GPIO_WritePin(CPU_LOAD_GPIO_Port, CPU_LOAD_Pin, GPIO_PIN_RESET);
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Enables the Clock Security System 
    */
  HAL_RCCEx_EnableLSECSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_IDLE_PHASE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_384CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 39;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  LL_SPI_InitTypeDef SPI_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  
  /**SPI2 GPIO Configuration  
  PB13   ------> SPI2_SCK
  PB15   ------> SPI2_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI2 parameter configuration*/

   /* SPI parameters configuration */
  
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  LL_SPI_InitTypeDef SPI_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  
  /**SPI3 GPIO Configuration  
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  PC12   ------> SPI3_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 parameter configuration*/

   /* SPI parameters configuration */
  
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI3, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 31;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim10, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 31000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim11, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_RST_Pin|OLED_DC_Pin|OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CPU_LOAD_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_OK_Pin */
  GPIO_InitStruct.Pin = BUTTON_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_OK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_CANCEL_Pin BUTTON_LEFT_Pin BUTTON_RIGHT_Pin */
  GPIO_InitStruct.Pin = BUTTON_CANCEL_Pin|BUTTON_LEFT_Pin|BUTTON_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_RST_Pin OLED_DC_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_RST_Pin|OLED_DC_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CPU_LOAD_Pin */
  GPIO_InitStruct.Pin = CPU_LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CPU_LOAD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CD_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
static void EnterSTOPMode(void)
{
	stopFlag = 1;

	OLED_DisplayOFF();
	user_bmp280_sleep_mode();
	UBX_powerSavingON();

	HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

static void LeaveSTOPMode(void)
{
   stopFlag = 0;
   ignoreFirstPress = 1;

   HAL_ResumeTick();
   SystemClock_Config();
   SystemCoreClock = 32000000UL;
   SysTick_Config(SystemCoreClock / TICKS_PER_SEC);

   UBX_powerSavingOFF();
   OLED_DisplayON();
}


// ***************** STATEMACHINE SECTION START *****************
uint8_t RotateValue(uint8_t value, int8_t step, uint8_t count)
{
	if(step == 1)
	{
		return (value + 1) % count;
	}
	else if(step == -1)
	{
		if(value == 0)
			return count - 1;
		else
			return (value - 1) % count;
	}
	else
	{
		return value;
	}
}

static MENU SwitchStateSettings(MENU menu, BUTTONS *keys, BUTTONS *oldKeys)
{
	static uint16_t oldSetting = 0;
	if(menu.screen >= 0 && menu.screen < 3) //main settings list
	{
		if(KeyPressed(keys->left, oldKeys->left))          //back
		{
			menu.screen = RotateValue(menu.screen, -1, 3);
		}
		else if(KeyPressed(keys->right, oldKeys->right))   //next
		{
			menu.screen = RotateValue(menu.screen, 1, 3);
		}
		else if(KeyPressed(keys->cancel, oldKeys->cancel)) //back to main menu
		{
			menu.menu = 0;
			menu.screen = 0;
		}
		else if(KeyPressed(keys->ok, oldKeys->ok))
		{
			if(menu.screen == 0)
			{
				oldSetting = Ride_GetRider().weight; //save old rider weight
				menu.screen = 3;
			}
			else if(menu.screen == 1)
			{
				oldSetting = Ride_GetBike().type; //save old bike type
				menu.screen = 6 + Ride_GetBike().type;
			}
			else if(menu.screen == 2)
			{
				oldSetting = Ride_GetBike().weight; //save old bike weight
				menu.screen = 6 + BIKE_COUNT;
			}
		}
	}
	else if(menu.screen >= 3 && menu.screen < 6) // additional menu select each digit of weight
	{
		const uint16_t scales[] = {1000,100,10};
		uint16_t val = scales[menu.screen - 3];

		if(KeyPressed(keys->left, oldKeys->left)) //decrement
		{
			RIDER rider = Ride_GetRider();
			if(rider.weight < val)
				rider.weight = (10000 - val) + rider.weight;
			else
				rider.weight = rider.weight - val;
			Ride_SetSettings(rider,Ride_GetBike());
		}
		else if(KeyPressed(keys->right, oldKeys->right)) //increment
		{
			RIDER rider = Ride_GetRider();
			rider.weight = (rider.weight + val) % 10000;
			Ride_SetSettings(rider,Ride_GetBike());
		}
		else if(KeyPressed(keys->cancel, oldKeys->cancel)) //move cursor backward
		{
			if(menu.screen == 3)
			{ //restore rider weight and go back
				RIDER rider = Ride_GetRider(); //so that adding fields wont break anything
				rider.weight = oldSetting;
				Ride_SetSettings(rider,Ride_GetBike());
				menu.screen = 0;
			}
			else
				menu.screen = menu.screen - 1;
		}
		else if(KeyPressed(keys->ok, oldKeys->ok)) //move cursor backward
		{
			if(menu.screen == 5)
				menu.screen = 0;
			else
				menu.screen = menu.screen + 1;
		}
	}
	else if(menu.screen >= 6 && menu.screen < (6+BIKE_COUNT)) //additional bike type select
	{
		if(KeyPressed(keys->left, oldKeys->left)) //decrement
		{
			menu.screen = RotateValue(menu.screen - 6,-1,BIKE_COUNT) + 6;
			BIKE bike = Ride_GetBike();
			bike.type = menu.screen - 6;
			Ride_SetSettings(Ride_GetRider(),bike);
		}
		else if(KeyPressed(keys->right, oldKeys->right)) //increment
		{
			menu.screen = RotateValue(menu.screen - 6,1,BIKE_COUNT) + 6;
			BIKE bike = Ride_GetBike();
			bike.type = menu.screen - 6;
			Ride_SetSettings(Ride_GetRider(),bike);
		}
		else if(KeyPressed(keys->cancel, oldKeys->cancel))	// to main list
		{ //restore bike type and go back
			BIKE bike = Ride_GetBike();
			bike.type = (enum BikeType) oldSetting; //valid cast to enum
			Ride_SetSettings(Ride_GetRider(),bike);
			menu.screen = 1;
		}
		else if(KeyPressed(keys->ok, oldKeys->ok)) // to main list
		{
			menu.screen = 1;
		}
	}
	else if(menu.screen >= (6+BIKE_COUNT) && menu.screen < (9+BIKE_COUNT)) //bike weight select
	{
		const uint16_t scales[] = {100,10,1};
		uint16_t val = scales[menu.screen - (6+BIKE_COUNT)];

		if(KeyPressed(keys->left, oldKeys->left)) //decrement
		{
			BIKE bike = Ride_GetBike();
			if(bike.weight < val) bike.weight = (1000 - val) + bike.weight;
			else bike.weight = bike.weight - val;
			Ride_SetSettings(Ride_GetRider(), bike);
		}
		else if(KeyPressed(keys->right, oldKeys->right)) //increment
		{
			BIKE bike = Ride_GetBike();
			bike.weight = (bike.weight + val) % 1000;
			Ride_SetSettings(Ride_GetRider(), bike);
		}
		else if(KeyPressed(keys->cancel, oldKeys->cancel)) //move cursor backward
		{
			if(menu.screen == (6+BIKE_COUNT))
			{ //restore bike weight and go back
				BIKE bike = Ride_GetBike();
				bike.weight = oldSetting;
				Ride_SetSettings(Ride_GetRider(),bike);
				menu.screen = 2;
			}
			else
				menu.screen = menu.screen - 1;
		}
		else if(KeyPressed(keys->ok, oldKeys->ok)) //move cursor backward
		{
			if(menu.screen == (8+BIKE_COUNT))
				menu.screen = 2;
			else
				menu.screen = menu.screen + 1;
		}
	}
	return menu;
}

static MENU StateMachine(MENU menu, BUTTONS *keys, BUTTONS *oldKeys, uint8_t menuScreens[])
{
	static char *currentFile = NULL;
	static uint8_t lastMenuScreen;

	if (notification.type == WAITING_FOR_GPS)
	{
		if ((gpsData.hdop >= PRECISION_THRESHOLD) && (lastGpsData.hdop >= PRECISION_THRESHOLD))
		{
			if (autopausedFlag)
			{
				autopausedFlag = 0;
				notification.type = NONE;
				notification.duration = 0;
				Ride_SetState(STARTED);
			}
			else
			{
				notification.type = NONE;
				notification.duration = 0;
			}
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			notification.type = NONE;
			notification.duration = 0;
			menu.menu = 0;
			menu.screen = 0;
		}

		if (keys->ok.holdCount >= STOP_MODE_TICKS)
		{
			keys->ok.holdCount = 0;
			ignoreFirstPress = 1;

			notification.type = NONE;
			notification.duration = 0;
			menu.menu = 8;
			menu.screen = 1;
		}

		return menu;
	}
	else if (notification.type != NONE)
		return menu;

	if (menu.menu == 0) // main menu
	{
		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (menu.screen == 0)
				menu.screen = menuScreens[menu.menu] - 1;
			else
				menu.screen = (menu.screen - 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			menu.screen = (menu.screen + 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->ok, oldKeys->ok))
		{
			if (!ignoreFirstPress)
			{
				if (menu.screen == 0)
				{
					if (Ride_GetState() == STOPPED)
					{
						fileResult = WriteHeaderToSD();
						if (fileResult != FR_OK)
						{
							notification.type = SD_CARD_ERROR;
							notification.duration = 3000;
						}
						Ride_SetState(PAUSED);
					}
					menu.menu = menu.screen + 1;
					menu.screen = 0;
				}
				else if (menu.screen == 3)
				{
					fileResult = ScanDirectoryOnSD("");
					if (fileResult != FR_OK)
					{
						notification.type = SD_CARD_ERROR;
						notification.duration = 3000;
						menu.menu = 0;
						menu.screen = 3;
					}
					else if (!fileNumber)
					{
						notification.type = NO_FILES_FOUND;
						notification.duration = 2000;
						menu.menu = 0;
						menu.screen = 3;
					}
					else
					{
						menuScreens[4] = fileNumber;
						menu.menu = menu.screen + 1;
						menu.screen = 0;
					}
				}
				else
				{
					menu.menu = menu.screen + 1;
					menu.screen = 0;
				}
			}
			else
			{
				menu.menu = 0;
				menu.screen = 0;
				user_bmp280_normal_mode();
				ignoreFirstPress = 0;
			}
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			menu.menu = 0;
			menu.screen = 0;
		}

		if (keys->cancel.holdCount >= STOP_MODE_TICKS)
		{
			Ride_SetState(PAUSED);
			EnterSTOPMode();
			keys->cancel.holdCount = 0;
		}
	}
	else if (menu.menu == 1)
	{
		if ((gpsData.hdop < PRECISION_THRESHOLD) || (lastGpsData.hdop < PRECISION_THRESHOLD))
		{
			notification.type = WAITING_FOR_GPS;
			notification.duration = INFINITE;
			if (Ride_GetState() != PAUSED && !autopausedFlag)
			{
				autopausedFlag = 1;
				Ride_SetState(PAUSED);
			}
		}
		else if (gpsData.velocity < AUTOPAUSE_VELOCITY && lastGpsData.velocity < AUTOPAUSE_VELOCITY)
		{
			if ((Ride_GetState() == STARTED) && !autopausedFlag)
			{
				notification.type = AUTOPAUSE;
				notification.duration = 1000;
				Ride_SetState(PAUSED);
				autopausedFlag = 1;
			}
		}
		else if (gpsData.velocity > AUTOSTART_VELOCITY && lastGpsData.velocity > AUTOSTART_VELOCITY)
		{
			if (autopausedFlag)
			{
				notification.type = START;
				notification.duration = 1000;
				Ride_SetState(STARTED);
				autopausedFlag = 0;
			}
		}

		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (menu.screen == 0)
				menu.screen = menuScreens[menu.menu] - 1;
			else
				menu.screen = (menu.screen - 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			menu.screen = (menu.screen + 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->ok, oldKeys->ok))
		{
			if (Ride_GetState() == STARTED)
			{
				Ride_SetState(PAUSED);
				notification.type = PAUSE;
				notification.duration = 1000;
				autopausedFlag = 0;
			}
			else if (Ride_GetState() == PAUSED)
			{
				if (!autopausedFlag)
				{
					Ride_SetState(STARTED);
					notification.type = START;
					notification.duration = 1000;
				}
			}
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			if (Ride_GetState() == STARTED)
			{
				Ride_SetState(PAUSED);
				notification.type = PAUSE;
				notification.duration = 1000;
			}
			autopausedFlag = 0;
			menu.menu = 0;
			menu.screen = 0;
		}

		if (keys->ok.holdCount >= STOP_MODE_TICKS)
		{
			keys->ok.holdCount = 0;
			ignoreFirstPress = 1;

			menu.menu = 8;
			menu.screen = 1;
		}
	}
	else if (menu.menu == 2)
	{
		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (menu.screen == 0)
				menu.screen = menuScreens[menu.menu] - 1;
			else
				menu.screen = (menu.screen - 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			menu.screen = (menu.screen + 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			menu.menu = 0;
			menu.screen = 0;
		}
	}
	else if (menu.menu == 3)
	{
		oldTimezone = timezone;

		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (menu.screen == 0)
				menu.screen = menuScreens[menu.menu] - 1;
			else
				menu.screen = (menu.screen - 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			menu.screen = (menu.screen + 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->ok, oldKeys->ok))
		{
			timeSet.tm_year = sDate.Year + 80;
			timeSet.tm_mon = sDate.Month - 1;
			timeSet.tm_mday = sDate.Date;
			timeSet.tm_hour = sTime.Hours;
			timeSet.tm_min = sTime.Minutes;
			timeSet.tm_sec = sTime.Seconds;

			if (menu.screen == 0)
			{
				menu.menu = 6;
				menu.screen = 0;
			}
			else
			{
				menu.menu = 7;
				menu.screen = 0;
			}
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			menu.menu = 0;
			menu.screen = 0;
		}
	}
	else if (menu.menu == 4)
	{
		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (menu.screen > 0)
				menu.screen--;
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			if (menu.screen < (menuScreens[menu.menu] - 1))
				menu.screen++;
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			menu.menu = 0;
			menu.screen = 0;
		}

		if (keys->ok.holdCount >= DELETE_FILE_TICKS)
		{
			keys->ok.holdCount = 0;
			ignoreFirstPress = 1;

			currentFile = fileList[menu.screen];
			lastMenuScreen = menu.screen;

			menu.menu = 9;
			menu.screen = 1;
		}
	}
	else if (menu.menu == 5)
	{
		menu = SwitchStateSettings(menu,keys,oldKeys);
	}
	else if (menu.menu == 6)
	{
		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (timezone > -12)
				timezone--;
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			if (timezone < 12)
				timezone++;
		}

		if (KeyPressed(keys->ok, oldKeys->ok))
		{
			if (gpsData.hdop >= PRECISION_THRESHOLD)
			{
				uint32_t gpsUTCDate;
				uint32_t gpsUTCTime;

				gpsUTCDate = gpsData.date;
				gpsUTCTime = (uint32_t)(gpsData.utcTime);

				timeSet.tm_year = 20 + (gpsUTCDate % 100);
				timeSet.tm_mon = (gpsUTCDate / 100) % 100;
				timeSet.tm_mday = (gpsUTCDate / 10000);
				timeSet.tm_hour = (gpsUTCTime / 10000) + timezone;
				timeSet.tm_min = (gpsUTCTime / 100) % 100;
				timeSet.tm_sec = (gpsUTCTime % 100);
				mktime(&timeSet);
				sDate.Year = timeSet.tm_year;
				sDate.Month = timeSet.tm_mon;
				sDate.Date = timeSet.tm_mday;
				sTime.Hours = timeSet.tm_hour;
				sTime.Minutes = timeSet.tm_min;
				sTime.Seconds = timeSet.tm_sec;
				HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			}
			else
			{
				timezone = oldTimezone;

				notification.type = WEAK_GPS_SIGNAL;
				notification.duration = 2000;
			}

			menu.menu = 3;
			menu.screen = 0;
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			timezone = oldTimezone;

			menu.menu = 3;
			menu.screen = 0;
		}
	}
	else if (menu.menu == 7)
	{
		if (KeyPressed(keys->left, oldKeys->left))
		{
			switch (menu.screen)
			{
				case 0:
					timeSet.tm_year = timeSet.tm_year - 1000;
					break;
				case 1:
					timeSet.tm_year = timeSet.tm_year - 100;
					break;
				case 2:
					timeSet.tm_year = timeSet.tm_year - 10;
					break;
				case 3:
					timeSet.tm_year = timeSet.tm_year - 1;
					break;
				case 4:
					timeSet.tm_mon = timeSet.tm_mon - 10;
					break;
				case 5:
					timeSet.tm_mon = timeSet.tm_mon - 1;
					break;
				case 6:
					timeSet.tm_mday = timeSet.tm_mday - 10;
					break;
				case 7:
					timeSet.tm_mday = timeSet.tm_mday - 1;
					break;
				case 8:
					timeSet.tm_hour = timeSet.tm_hour - 10;
					break;
				case 9:
					timeSet.tm_hour = timeSet.tm_hour - 1;
					break;
				case 10:
					timeSet.tm_min = timeSet.tm_min - 10;
					break;
				case 11:
					timeSet.tm_min = timeSet.tm_min - 1;
					break;
				case 12:
					timeSet.tm_sec = timeSet.tm_sec - 10;
					break;
				case 13:
					timeSet.tm_sec = timeSet.tm_sec - 1;
					break;
				case 14:
					if (timezone > -12)
						timezone--;
					break;
				default:
					break;
			}
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			switch (menu.screen)
			{
				case 0:
					timeSet.tm_year = timeSet.tm_year + 1000;
					break;
				case 1:
					timeSet.tm_year = timeSet.tm_year + 100;
					break;
				case 2:
					timeSet.tm_year = timeSet.tm_year + 10;
					break;
				case 3:
					timeSet.tm_year = timeSet.tm_year + 1;
					break;
				case 4:
					timeSet.tm_mon = timeSet.tm_mon + 10;
					break;
				case 5:
					timeSet.tm_mon = timeSet.tm_mon + 1;
					break;
				case 6:
					timeSet.tm_mday = timeSet.tm_mday + 10;
					break;
				case 7:
					timeSet.tm_mday = timeSet.tm_mday + 1;
					break;
				case 8:
					timeSet.tm_hour = timeSet.tm_hour + 10;
					break;
				case 9:
					timeSet.tm_hour = timeSet.tm_hour + 1;
					break;
				case 10:
					timeSet.tm_min = timeSet.tm_min + 10;
					break;
				case 11:
					timeSet.tm_min = timeSet.tm_min + 1;
					break;
				case 12:
					timeSet.tm_sec = timeSet.tm_sec + 10;
					break;
				case 13:
					timeSet.tm_sec = timeSet.tm_sec + 1;
					break;
				case 14:
					if (timezone < 12)
						timezone++;
					break;
				default:
					break;
			}
		}

		if (timeSet.tm_year < 80)
		{
			timeSet.tm_year = 80;
			timeSet.tm_mon = 0;
			timeSet.tm_mday = 1;
			timeSet.tm_hour = 0;
			timeSet.tm_min = 0;
			timeSet.tm_sec = 0;
		}
		mktime(&timeSet);

		if (KeyPressed(keys->ok, oldKeys->ok))
		{
			if (menu.screen == (menuScreens[menu.menu] - 1))
			{
				sDate.Year = timeSet.tm_year - 80;
				sDate.Month = timeSet.tm_mon + 1;
				sDate.Date = timeSet.tm_mday;
				sTime.Hours = timeSet.tm_hour;
				sTime.Minutes = timeSet.tm_min;
				sTime.Seconds = timeSet.tm_sec;
				HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

				menu.menu = 3;
				menu.screen = 1;
			}
			else
			{
				menu.screen++;
			}
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			menu.menu = 3;
			menu.screen = 1;
		}
	}
	else if (menu.menu == 8)
	{
		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (menu.screen == 0)
				menu.screen = menuScreens[menu.menu] - 1;
			else
				menu.screen = (menu.screen - 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			menu.screen = (menu.screen + 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->ok, oldKeys->ok))
		{
			if (!ignoreFirstPress)
			{
				if (menu.screen == 0)
				{
					Ride_ClearInfo();
					Ride_SetState(STOPPED);
					if (fileResult == FR_OK)
					{
						fileResult = WriteEndingToSD();
					}
					notification.type = STOP;
					notification.duration = 2000;
					autopausedFlag = 0;
					menu.menu = 0;
					menu.screen = 0;
				}
				else
				{
					menu.menu = 1;
					menu.screen = 0;
				}
			}
			else
			{
				ignoreFirstPress = 0;
			}
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			menu.menu = 1;
			menu.screen = 0;
		}
	}
	else if (menu.menu == 9)
	{
		if (KeyPressed(keys->left, oldKeys->left))
		{
			if (menu.screen == 0)
				menu.screen = menuScreens[menu.menu] - 1;
			else
				menu.screen = (menu.screen - 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->right, oldKeys->right))
		{
			menu.screen = (menu.screen + 1) % menuScreens[menu.menu];
		}

		if (KeyPressed(keys->ok, oldKeys->ok))
		{
			if (!ignoreFirstPress)
			{
				if (menu.screen == 0)
				{
					RemoveFileFromSD(currentFile);
					fileResult = ScanDirectoryOnSD("");
					if (fileResult != FR_OK)
					{
						notification.type = SD_CARD_ERROR;
						notification.duration = 3000;
						currentFile = NULL;
						menu.menu = 0;
						menu.screen = 3;
					}
					else if (!fileNumber)
					{
						notification.type = NO_FILES_FOUND;
						notification.duration = 2000;
						currentFile = NULL;
						menu.menu = 0;
						menu.screen = 3;
					}
					else
					{
						menuScreens[4] = fileNumber;
						currentFile = NULL;
						menu.menu = 4;
						menu.screen = 0;
					}
				}
				else
				{
					currentFile = NULL;
					menu.menu = 4;
					menu.screen = lastMenuScreen;
				}
			}
			else
			{
				ignoreFirstPress = 0;
			}
		}

		if (KeyPressed(keys->cancel, oldKeys->cancel))
		{
			menu.menu = 4;
			menu.screen = lastMenuScreen;
		}
	}

	if (!(keys->ok.state) & !(keys->cancel.state) & !(keys->left.state) & !(keys->right.state))
	{
	  if ((idleCounter >= (60 * STOP_MODE_MINUTES)) && (Ride_GetState() != STARTED))
	  {
		  EnterSTOPMode();
		  idleCounter = 0;
	  }
	}
	else
	{
		idleCounter = 0;
	}

	return menu;
}
// ***************** STATEMACHINE SECTION END *****************


static BATTERY ReadBatteryState(void)
{
	const int HISTERESIS = 10; // should be even number
	BATTERY battery;
	static int8_t lastLevel = -1;
	uint8_t nextLevel = 0;

	if (batteryVoltage >= BATTERY_VOLTAGE_MAX)
	{
		nextLevel = 100;
		if (batteryVoltage >= (BATTERY_VOLTAGE_MAX + 10))
			battery.state = BATTERY_ERROR;
		else
			battery.state = BATTERY_OK;
	}
	else if (batteryVoltage <= BATTERY_VOLTAGE_MIN)
	{
		nextLevel = 0;
		if (batteryVoltage <= (BATTERY_VOLTAGE_MIN - 10))
			battery.state = BATTERY_ERROR;
		else
			battery.state = BATTERY_DISCHARGED;
	}
	else
	{
		nextLevel = (100 * (batteryVoltage - BATTERY_VOLTAGE_MIN)) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN);
		if (nextLevel <= BATTERY_LEVEL_LOW)
			battery.state = BATTERY_LOW;
		else
			battery.state = BATTERY_OK;
	}

	if(lastLevel < 0)
	{
		lastLevel = (nextLevel + (HISTERESIS/2)) / HISTERESIS; //round to HISTERESIS %
	}
	else
	{
		if(abs((int)nextLevel - (int)lastLevel) >= HISTERESIS)
			lastLevel = (nextLevel + (HISTERESIS/2)) / HISTERESIS;
	}
	battery.level = lastLevel; //set rounded value

	return battery;
}

static BUTTONS ReadButtons(BUTTONS keys, BUTTONS oldKeys)
{
	keys.ok.state = (uint8_t)HAL_GPIO_ReadPin(BUTTON_OK_GPIO_Port, BUTTON_OK_Pin);
	keys.cancel.state = !(uint8_t)HAL_GPIO_ReadPin(BUTTON_CANCEL_GPIO_Port, BUTTON_CANCEL_Pin);
	keys.left.state = !(uint8_t)HAL_GPIO_ReadPin(BUTTON_LEFT_GPIO_Port, BUTTON_LEFT_Pin);
	keys.right.state = !(uint8_t)HAL_GPIO_ReadPin(BUTTON_RIGHT_GPIO_Port, BUTTON_RIGHT_Pin);

	keys.ok.holdCount = ReadButtonHoldCount(keys.ok, oldKeys.ok);
	keys.cancel.holdCount = ReadButtonHoldCount(keys.cancel, oldKeys.cancel);
	keys.left.holdCount = ReadButtonHoldCount(keys.left, oldKeys.left);
	keys.right.holdCount = ReadButtonHoldCount(keys.right, oldKeys.right);

	return keys;
}

static uint8_t ReadButtonHoldCount(BUTTON key, BUTTON oldkey)
{
	  if (KeyPressed(key, oldkey))
	  {
		  key.holdCount = 0;
	  }
	  else if (key.state)
	  {
		  key.holdCount++;
	  }

	  return key.holdCount;
}


// ***************** OLED DISPLAY SECTION START *****************
static void DisplayGPSSignal(const GPS_Data* gps)
{
	static uint8_t gpsIcon = 0;
	if ((gps->hdop == IDEAL) || (gps->hdop == EXCELLENT) || (gps->hdop == GOOD))
	{
		GUI_Disbitmap(2, 2, GPS_1616, 16, 16);
		gpsIcon = 1;
	}
	else if ((gps->hdop == MODERATE) || (gps->hdop == WEAK))
	{
		if (gpsIcon) GUI_Disbitmap(2, 2, GPS_1616, 16, 16);
		gpsIcon = !gpsIcon;
	}
	else
	{
		gpsIcon = 0;
	}
}

static void DisplayTime(const RTC_TimeTypeDef* time)
{
	char timeString[6];

	sprintf(timeString, "%02u:%02u", time->Hours, time->Minutes);
	GUI_DisString_EN(46, 6, timeString, &Font12, BLACK, WHITE);
}

static void DisplayBatteryLevel(BATTERY battery)
{
	if (battery.level > 80)
		GUI_Disbitmap(106, 6, battery80100_2011, 20, 11);
	else if (battery.level > 60)
		GUI_Disbitmap(106, 6, battery6080_2011, 20, 11);
	else if (battery.level > 40)
		GUI_Disbitmap(106, 6, battery4060_2011, 20, 11);
	else if (battery.level > 20)
		GUI_Disbitmap(106, 6, battery2040_2011, 20, 11);
	else if (battery.level > 5)
		GUI_Disbitmap(106, 6, battery0520_2011, 20, 11);
	else
		GUI_Disbitmap(106, 6, battery0005_2011, 20, 11);
}

static void DisplayHeader(BATTERY battery,const GPS_Data* gps,const RTC_TimeTypeDef* time)
{
	DisplayGPSSignal(gps);
	DisplayTime(time);
	DisplayBatteryLevel(battery);
	GUI_DrawLine(0, 25, 128, 25, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
}


static uint8_t DisplayNotification(NOTIFICATION not,GPS_Data* gd)
{
	if (not.type == WELCOME)
	{
		GUI_DisString_EN(14, 50, "CYCLING", &Font20, FONT_BACKGROUND, WHITE);
		GUI_DisString_EN(20, 70, "COMPUTER", &Font16, FONT_BACKGROUND, WHITE);
	}
	else if (not.type == START)
	{
		GUI_DisGrayMap(39, 40, gImage_start);
		GUI_DisString_EN(26, 100, "STARTED", &Font16, FONT_BACKGROUND, WHITE);
	}
	else if (not.type == STOP)
	{
		GUI_DisGrayMap(39, 40, gImage_stop);
		GUI_DisString_EN(26, 100, "STOPPED", &Font16, FONT_BACKGROUND, WHITE);
	}
	else if (not.type == PAUSE)
	{
		GUI_DisGrayMap(39, 40, gImage_pause);
		GUI_DisString_EN(32, 100, "PAUSED", &Font16, FONT_BACKGROUND, WHITE);
	}
	else if (not.type == AUTOPAUSE)
	{
		GUI_DisGrayMap(39, 40, gImage_pause);
		GUI_DisString_EN(8, 100, "AUTOPAUSED", &Font16, FONT_BACKGROUND, WHITE);
	}
	else if (not.type == WEAK_GPS_SIGNAL)
	{
		GUI_DisGrayMap(39, 40, gImage_alert);
		GUI_DisString_EN(10, 100, "Weak GPS Signal!", &Font12, FONT_BACKGROUND, WHITE);
	}
	else if (not.type == WAITING_FOR_GPS)
	{
		char satellitesString[20];

		sprintf(satellitesString, "Satellites: %2u", gd->satellitesCount);
		GUI_DisGrayMap(39, 40, gImage_gpsBig);
		GUI_DisString_EN(0, 100, "Waiting for GPS...", &Font12, FONT_BACKGROUND, WHITE);
		GUI_DisString_EN(15, 115, satellitesString, &Font12, FONT_BACKGROUND, WHITE);
	}
	else if (not.type == SD_CARD_ERROR)
	{
		GUI_DisGrayMap(39, 40, gImage_alert);
		GUI_DisString_EN(15, 100, "SD card error!", &Font12, FONT_BACKGROUND,
				WHITE);
	}
	else if (not.type == NO_FILES_FOUND)
	{
		GUI_DisGrayMap(39, 40, gImage_alert);
		GUI_DisString_EN(15, 100, "No files found!", &Font12, FONT_BACKGROUND, WHITE);
	}
	else
	{
		return 0;
	}
	return 1;
}

static uint16_t CalcCenterStrX(const char* pString, const sFONT* font)
{
	const int16_t SCREEN_WIDTH = 128;
	int16_t width = font->Width * strlen(pString);
	int16_t pos = SCREEN_WIDTH - width;
	if(pos < 0) pos = 0;
	return pos / 2;
}

static void DrawSetting(const char* desc,const char* value,char isTop,HIGHLIGHT hl,uint8_t selIndex)
{
	COLOR valBg = FONT_BACKGROUND;
	COLOR valFg = WHITE;

	if(hl == HILI_VALUE)
	{
		valBg = FONT_FOREGROUND;
		valFg = BLACK;
	}
	if(isTop)
	{
		uint16_t valX = CalcCenterStrX(value,&Font16);
		if(hl == HILI_VALUE_SQUARE)
		{
			uint16_t xlen = valX + Font16.Width*(strlen(value)-1);
			GUI_DrawRectangle(valX,55,xlen + 10,55 + 12, WHITE, DRAW_EMPTY, DOT_PIXEL_1X1);
		}
		else if(hl == HILI_INDEX)
		{
			uint16_t xpos = valX + Font16.Width*selIndex;
			GUI_DrawRectangle(xpos,55,xpos + 10,55 + 12, WHITE, DRAW_EMPTY, DOT_PIXEL_1X1);
		}
		GUI_DisString_EN(CalcCenterStrX(desc,&Font12),35,desc, &Font12, FONT_BACKGROUND, WHITE);
		GUI_DisString_EN(valX,55,value, &Font16, valBg, valFg);
	}
	else
	{
		uint16_t valX = CalcCenterStrX(value,&Font16);
		if(hl == HILI_VALUE_SQUARE)
		{
			uint16_t xlen = valX + Font16.Width*(strlen(value)-1);
			GUI_DrawRectangle(valX,110,xlen + 10,110 + 12, WHITE, DRAW_EMPTY, DOT_PIXEL_1X1);
		}
		else if(hl == HILI_INDEX)
		{
			uint16_t xpos = valX + Font16.Width*selIndex;
			GUI_DrawRectangle(xpos,110,xpos + 10,110 + 12, WHITE, DRAW_EMPTY, DOT_PIXEL_1X1);
		}
		GUI_DisString_EN(CalcCenterStrX(desc,&Font12),90,desc, &Font12, FONT_BACKGROUND, WHITE);
		GUI_DisString_EN(valX,110,value, &Font16, valBg, valFg);
	}
}

static void DisplaySettings(MENU menu)
{
	GUI_DrawLine(0, 80, 128, 80, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
	if(menu.screen == 0 || menu.screen == 1) // select weight or bike
	{
		char weightStr[10];

		const char* typeStr = bikeTypeArray[Ride_GetBike().type];
		sprintf(weightStr, "%3u kg", Ride_GetRider().weight/10);
		DrawSetting("Rider weight:",weightStr,1,menu.screen == 0 ? HILI_VALUE : HILI_NONE,0);
		DrawSetting("Bike type:",typeStr,0,menu.screen == 1 ? HILI_VALUE : HILI_NONE,0);
	}
	else if(menu.screen == 2) // select bike weight
	{
		char weightStr[10];

		sprintf(weightStr, "%2u.%u kg", Ride_GetBike().weight/10,Ride_GetBike().weight%10);
		DrawSetting("Bike weight:",weightStr,1,HILI_VALUE,0);
	}
	else if(menu.screen >= 3 && menu.screen < 6) // additional menu select each digit of weight
	{
		char weightStr[10];

		const char* typeStr = bikeTypeArray[Ride_GetBike().type];
		sprintf(weightStr, "%03u kg", Ride_GetRider().weight/10);
		DrawSetting("Rider weight:",weightStr,1,HILI_INDEX,menu.screen - 3);
		DrawSetting("Bike type:",typeStr,0,HILI_NONE,0);
	}
	else if(menu.screen >= 6 && menu.screen < (6+BIKE_COUNT)) // AM select type
	{
		char weightStr[10];

		sprintf(weightStr, "%3u kg", Ride_GetRider().weight/10);
		DrawSetting("Rider weight:",weightStr,1,HILI_NONE,0);
		DrawSetting("Bike type:",bikeTypeArray[menu.screen - 6],0,HILI_VALUE_SQUARE,0);
	}
	else if(menu.screen >= (6+BIKE_COUNT) && menu.screen < (9+BIKE_COUNT))
	{
		char weightStr[10];
		uint8_t select = menu.screen - (6+BIKE_COUNT);

		sprintf(weightStr, "%02u.%u kg", Ride_GetBike().weight/10,Ride_GetBike().weight%10);
		DrawSetting("Bike weight:",weightStr,1,HILI_INDEX,select > 1 ? select + 1 : select);
	}
	else
	{
		//unreachable
	}
}

static void Display(BATTERY battery, MENU menu)
{
	OLED_Clear(OLED_BACKGROUND);
	DisplayHeader(battery,&gpsData,&sTime);

	if(DisplayNotification(notification,&gpsData)) //if any notification is present then refresh right away
	{
		OLED_Display();
		return;
	}

	if (menu.menu == 0) //main menu
	{
		if (menu.screen == 0)
		{
			GUI_DisGrayMap(39, 40, gImage_bike);
			GUI_DisString_EN(42, 100, "RIDE", &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 1)
		{
			GUI_DisGrayMap(39, 40, gImage_gpsdata);
			GUI_DisString_EN(2, 100, "SENSOR DATA", &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 2)
		{
			GUI_DisGrayMap(39, 40, gImage_calendar);
			GUI_DisString_EN(42, 100, "DATE", &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 3)
		{
			GUI_DisGrayMap(39, 40, gImage_memory);
			GUI_DisString_EN(32, 100, "MEMORY", &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 4)
		{
			GUI_DisGrayMap(39, 40, gImage_settings);
			GUI_DisString_EN(20, 100, "SETTINGS", &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else
		{
			OLED_Display();
		}
	}
	else if (menu.menu == 1) //ride submenu
	{
		const RIDE* rideInfo = Ride_GetInfo();

		GUI_DrawLine(0, 80, 128, 80, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
		//GUI_DrawLine(60, 80, 60, 128, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
		if (menu.screen == 0)
		{
			char speedString[20];
			char slopeString[20];
			char powerString[20];

			sprintf(speedString, "%3u.%1u", (rideInfo->speed / 10), (rideInfo->speed % 10));
			sprintf(slopeString, "%2d.%1d%%", ((rideInfo->slope / 10) % 100), abs(rideInfo->slope % 10));
			sprintf(powerString, "%4u", rideInfo->power);
			GUI_DisString_EN(46, 35, "Speed", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(24, 55, speedString, &Font20, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(14, 90, "Slope", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(8, 110, slopeString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(80, 90, "Power", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(72, 110, powerString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 1)
		{
			char timeString[20] = "00:00:00";
			char distanceString[20];
			char elevGainString[20];

			sprintf(timeString, "%2u:%02u:%02u", (rideInfo->time.tm_yday * 24U) + rideInfo->time.tm_hour, rideInfo->time.tm_min, rideInfo->time.tm_sec);
			sprintf(distanceString, "%3lu.%1lu", (rideInfo->distance / 1000), ((rideInfo->distance % 1000) / 100));
			sprintf(elevGainString, "%4lu", (rideInfo->elevGain / 100));
			GUI_DisString_EN(50, 35, "Time", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(5, 55, timeString, &Font20, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(2, 90, "Distance", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(0, 110, distanceString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(64, 90, "Elev.Gain", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(72, 110, elevGainString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 2)
		{
			char avgSpeedString[20];
			char caloriesString[20];
			char avgPowerString[20];

			sprintf(avgSpeedString, "%3u.%1u", (rideInfo->avgSpeed / 10), (rideInfo->avgSpeed % 10));
			sprintf(avgPowerString, "%4u", rideInfo->avgPower);
			sprintf(caloriesString, "%4lu", (rideInfo->calories / 1000));
			GUI_DisString_EN(35, 35, "Avg.Speed", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(24, 55, avgSpeedString, &Font20, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(2, 90, "Calories", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(4, 110, caloriesString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(64, 90, "Avg.Power", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(72, 110, avgPowerString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else
		{
			OLED_Display();
		}
	}
	else if (menu.menu == 2) //sensor data submenu
	{
		GUI_DrawLine(0, 80, 128, 80, WHITE, LINE_SOLID, DOT_PIXEL_1X1);
		if (menu.screen == 0)
		{
			char latitudeString[30];
			char longitudeString[30];

			sprintf(latitudeString, "%02u.%02u'%02u\" %c", ((uint16_t)gpsData.latitude / 100), ((uint16_t)gpsData.latitude % 100),
					((uint16_t)(gpsData.latitude * 60) % 100), gpsData.latitudeHemisphere);
			sprintf(longitudeString, "%02u.%02u'%02u\" %c", ((uint16_t)gpsData.longitude / 100), ((uint16_t)gpsData.longitude % 100),
					((uint16_t)(gpsData.longitude * 600) % 100), gpsData.longitudeHemisphere);
			GUI_DisString_EN(34, 35, "Latitude:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(5, 55, latitudeString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(34, 90, "Longitude:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(5, 110, longitudeString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 1)
		{
			char altitudeString[20];
			char satellitesString[20];

			sprintf(altitudeString, "%.1f m", gpsData.altitude);
			sprintf(satellitesString, "%2u", gpsData.satellitesCount);
			GUI_DisString_EN(20, 35, "Altitude GPS:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(30, 55, altitudeString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(26, 90, "Satellites:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(48, 110, satellitesString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 2)
		{
			char velocityString[20];
			char courseString[20];

			sprintf(velocityString, "%.1f km/h", gpsData.velocity);
			sprintf(courseString, "%03.0f", gpsData.course);
			GUI_DisString_EN(34, 35, "Velocity:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(20, 55, velocityString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(40, 90, "Course:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(46, 110, courseString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 3)
		{
			char temperatureString[20];
			char atmosphericPressureString[20];
			uint32_t atmosphericPressure;

			atmosphericPressure = read_pressure_atmospheric();
			sprintf(temperatureString, "%2ld.%02d C", (barometerData.temperature / 100), abs(barometerData.temperature % 100));
			sprintf(atmosphericPressureString, "%4lu.%02lu hPa", (atmosphericPressure / 100), (atmosphericPressure % 100));
			GUI_DisString_EN(24, 35, "Temperature:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(28, 55, temperatureString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(18, 90, "Atm. pressure:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(2, 110, atmosphericPressureString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 4)
		{
			char altitudeString[20];
			char pressureString[20];

			sprintf(altitudeString, "%3ld.%01d m", (barometerData.altitude / 100), abs(((barometerData.altitude) % 100) / 10));
			sprintf(pressureString, "%4lu.%02lu hPa", (barometerData.pressure / 100), (barometerData.pressure % 100));
			GUI_DisString_EN(34, 35, "Altitude:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(30, 55, altitudeString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(34, 90, "Pressure:", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(2, 110, pressureString, &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else
		{
			OLED_Display();
		}
	}
	else if (menu.menu == 3) //date submenu
	{
		char dateString[11];
		char timeString[9];

		sprintf(dateString, "%04u-%02u-%02u", (1980 + sDate.Year), sDate.Month, sDate.Date);
		sprintf(timeString, "%02u:%02u:%02u", sTime.Hours, sTime.Minutes, sTime.Seconds);
		GUI_DisString_EN(5, 40, dateString, &Font16, BLACK, WHITE);
		GUI_DisString_EN(16, 60, timeString, &Font16, BLACK, WHITE);
		if (menu.screen == 0)
		{
			GUI_DisString_EN(5, 85, "Set automatically", &Font12, FONT_FOREGROUND, BLACK);
			GUI_DisString_EN(5, 105, "Set manually", &Font12, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else if (menu.screen == 1)
		{
			GUI_DisString_EN(5, 85, "Set automatically", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(5, 105, "Set manually", &Font12, FONT_FOREGROUND, BLACK);
			OLED_Display();
		}
		else
		{
			OLED_Display();
		}
	}
	else if (menu.menu == 4) // memory submenu
	{
		char fileNumberString[10];

		sprintf(fileNumberString, "%2u/%u", (menu.screen + 1), fileNumber);
		if ((menu.screen % 3) == 0)
		{
			GUI_DisString_EN(0, 35, *(fileList + menu.screen), &Font12, FONT_FOREGROUND, BLACK);
			if (fileNumber > (menu.screen + 1))
			{
				GUI_DisString_EN(0, 65, *(fileList + menu.screen + 1), &Font12, FONT_BACKGROUND, WHITE);
			}
			if (fileNumber > (menu.screen + 2))
			{
				GUI_DisString_EN(0, 95, *(fileList + menu.screen + 2), &Font12, FONT_BACKGROUND, WHITE);
			}
		}
		else if ((menu.screen % 3) == 1)
		{
			GUI_DisString_EN(0, 35, *(fileList + menu.screen - 1), &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(0, 65, *(fileList + menu.screen), &Font12, FONT_FOREGROUND, BLACK);
			if (fileNumber > (menu.screen + 1))
			{
				GUI_DisString_EN(0, 95, *(fileList + menu.screen + 1), &Font12, FONT_BACKGROUND, WHITE);
			}
		}
		else
		{
			GUI_DisString_EN(0, 35, *(fileList + menu.screen - 2), &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(0, 65, *(fileList + menu.screen - 1), &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(0, 95, *(fileList + menu.screen), &Font12, FONT_FOREGROUND, BLACK);
		}
		GUI_DisString_EN(92, 115, fileNumberString, &Font12, FONT_BACKGROUND, WHITE);
		OLED_Display();
	}
	else if (menu.menu == 5) //settings
	{
		DisplaySettings(menu);
		OLED_Display();
	}
	else if (menu.menu == 6) // date->timezone submenu
	{
		char timezoneString[20];

		sprintf(timezoneString, "UTC%+02dh", timezone);
		GUI_DisString_EN(18, 40, "Timezone:", &Font16, FONT_BACKGROUND, WHITE);
		GUI_DisString_EN(24, 70, timezoneString, &Font16, FONT_FOREGROUND, BLACK);
		OLED_Display();
	}
	else if (menu.menu == 7) //
	{
		char dateString[11];
		char timeString[9];
		char timezoneString[20];
		CURSOR cursorPosition;

		if (menu.screen < 4)
		{
			cursorPosition.x = 5 + (11 * menu.screen);
			cursorPosition.y = 40;
		}
		else if (menu.screen < 6)
		{
			cursorPosition.x = 5 + 11 + (11 * menu.screen);
			cursorPosition.y = 40;
		}
		else if (menu.screen < 8)
		{
			cursorPosition.x = 5 + (2 * 11) + (11 * menu.screen);
			cursorPosition.y = 40;
		}
		else if (menu.screen < 10)
		{
			cursorPosition.x = 16 + (11 * (menu.screen - 8));
			cursorPosition.y = 60;
		}
		else if (menu.screen < 12)
		{
			cursorPosition.x = 16 + 11 + (11 * (menu.screen - 8));
			cursorPosition.y = 60;
		}
		else if (menu.screen < 14)
		{
			cursorPosition.x = 16 + (2 * 11) + (11 * (menu.screen - 8));
			cursorPosition.y = 60;
		}

		if (menu.screen == 14)
		{
			sprintf(timezoneString, "UTC%+02dh", timezone);
			GUI_DisString_EN(18, 40, "Timezone:", &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(24, 70, timezoneString, &Font16, FONT_FOREGROUND, BLACK);
		}
		else
		{
			GUI_DrawRectangle(cursorPosition.x, cursorPosition.y, (cursorPosition.x + 10), (cursorPosition.y + 12), WHITE, DRAW_EMPTY, DOT_PIXEL_1X1);
			sprintf(dateString, "%04u-%02u-%02u", (1900 + timeSet.tm_year), (timeSet.tm_mon + 1), timeSet.tm_mday);
			sprintf(timeString, "%02u:%02u:%02u", timeSet.tm_hour, timeSet.tm_min, timeSet.tm_sec);
			GUI_DisString_EN(5, 40, dateString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(16, 60, timeString, &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(5, 85, "Set automatically", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(5, 105, "Set manually", &Font12, FONT_BACKGROUND, WHITE);
		}
		OLED_Display();
	}
	else if (menu.menu == 8)
	{
		if (menu.screen == 0)
		{
			GUI_DisString_EN(10, 40, "Are you sure you", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(20, 55, "want to stop?", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(25, 80, "YES", &Font16, FONT_FOREGROUND, BLACK);
			GUI_DisString_EN(85, 80, "NO", &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else
		{
			GUI_DisString_EN(10, 40, "Are you sure you", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(20, 55, "want to stop?", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(25, 80, "YES", &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(85, 80, "NO", &Font16, FONT_FOREGROUND, BLACK);
			OLED_Display();
		}
	}
	else if (menu.menu == 9)
	{
		if (menu.screen == 0)
		{
			GUI_DisString_EN(10, 40, "Are you sure you", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(14, 55, "want to delete?", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(25, 80, "YES", &Font16, FONT_FOREGROUND, BLACK);
			GUI_DisString_EN(85, 80, "NO", &Font16, FONT_BACKGROUND, WHITE);
			OLED_Display();
		}
		else
		{
			GUI_DisString_EN(10, 40, "Are you sure you", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(14, 55, "want to delete?", &Font12, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(25, 80, "YES", &Font16, FONT_BACKGROUND, WHITE);
			GUI_DisString_EN(85, 80, "NO", &Font16, FONT_FOREGROUND, BLACK);
			OLED_Display();
		}
	}
	else
	{
		OLED_Display();
	}
}
// ***************** OLED DISPLAY SECTION END *****************


static FRESULT WriteHeaderToSD(void)
{
	FRESULT fr;

	sprintf(currentGPXPath, "track_%04u%02u%02u_%02u%02u%02u.gpx", (1980 + sDate.Year), sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

	fr = f_mount(&fileSystem, "", 1);
	if (fr == FR_OK)
	{
	    fr = f_open(&file, (char*)currentGPXPath, FA_WRITE | FA_CREATE_ALWAYS);
	    if (fr == FR_OK)
	    {
			f_puts("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\n\n", &file);
			f_puts("<gpx xmlns=\"http://www.topografix.com/GPX/1/0\" version=\"1.0\" creator=\"CyclingComputer\"\n", &file);
			f_puts("xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">\n", &file);
			f_puts("<trk>\n", &file);
			f_puts("<trkseg>\n", &file);
	    	fr = f_close(&file);
	    }
	}

    return fr;
}

static FRESULT WriteTrackpointToSD(void)
{
	FRESULT fr;
	float latitude, longitude;
	float degrees, minutes;
	char coordinatesString[50];

	if (Ride_GetState() != STARTED)
	{
		return FR_OK;
	}

	latitude = gpsData.latitude / 100.0;
	minutes = 100.0 * modff(latitude, &degrees) / 60.0;
	latitude = degrees + minutes;
	if (gpsData.latitudeHemisphere == 'S')
		latitude = -latitude;

	longitude = gpsData.longitude / 100.0;
	minutes = 100.0 * modff(longitude, &degrees) / 60.0;
	longitude = degrees + minutes;
	if (gpsData.longitudeHemisphere == 'W')
		longitude = -longitude;

	sprintf(coordinatesString, "<trkpt lat=\"%.6f\" lon=\"%.6f\">\n", latitude, longitude);

	timeSet.tm_year = sDate.Year + 80;
	timeSet.tm_mon = sDate.Month - 1;
	timeSet.tm_mday = sDate.Date;
	timeSet.tm_hour = sTime.Hours;
	timeSet.tm_min = sTime.Minutes;
	timeSet.tm_sec = sTime.Seconds;

	timeSet.tm_hour -= timezone;
	if (timeSet.tm_year < 80)
	{
		timeSet.tm_year = 80;
		timeSet.tm_mon = 0;
		timeSet.tm_mday = 1;
		timeSet.tm_hour = 0;
		timeSet.tm_min = 0;
		timeSet.tm_sec = 0;
	}
	mktime(&timeSet);

	fr = f_mount(&fileSystem, "", 1);
	if (fr == FR_OK)
	{
	    fr = f_open(&file, (char*)currentGPXPath, FA_WRITE | FA_OPEN_EXISTING);
	    if (fr == FR_OK)
	    {
	    	fr = f_lseek(&file, f_size(&file));
	    	if (fr == FR_OK)
	    	{
	    		f_puts(coordinatesString, &file);
	    		f_printf(&file, "<ele>%ld.%02d</ele>\n", (barometerData.altitude / 100), abs((barometerData.altitude) % 100));
	    		f_printf(&file, "<time>%04u-%02u-%02uT%02u:%02u:%02uZ</time>\n", (1900 + timeSet.tm_year), (1 + timeSet.tm_mon), \
	    				timeSet.tm_mday, timeSet.tm_hour, timeSet.tm_min, timeSet.tm_sec);
	    		f_puts("</trkpt>\n", &file);
	    	}
	    	fr = f_close(&file);
	    }
	}

    return fr;
}

static FRESULT WriteEndingToSD(void)
{
	FRESULT fr;

	fr = f_mount(&fileSystem, "", 1);
	if (fr == FR_OK)
	{
	    fr = f_open(&file, (char*)currentGPXPath, FA_WRITE | FA_OPEN_EXISTING);
	    if (fr == FR_OK)
	    {
	    	fr = f_lseek(&file, f_size(&file));
	    	if (fr == FR_OK)
	    	{
	    		f_puts("</trkseg>\n", &file);
	    		f_puts("</trk>\n", &file);
	    		f_puts("</gpx>\n", &file);
	    	}
	    	fr = f_close(&file);
	    }
	}

    return fr;
}

static FRESULT ScanDirectoryOnSD(char* path)
{
    FRESULT res;
    DIR dir;
    FILINFO fno;
    char* fileName;

    res = f_mount(&fileSystem, "", 1);
	if (res == FR_OK)
	{
		res = f_opendir(&dir, path);							/* Open the directory */
		if (res == FR_OK)
		{
			ClearFileList();
			fno.lfsize = UINT8_MAX + 1;
			fno.lfname = (TCHAR*)malloc(fno.lfsize);
			if (fno.lfname == NULL)
			{
				f_closedir(&dir);
				return FR_OK;
			}
			for (;;)
			{
				res = f_readdir(&dir, &fno);                   	/* Read a directory item */
				if (res != FR_OK || fno.fname[0] == 0)
					break;  									/* Break on error or end of dir */
				if (fno.fattrib & AM_DIR)						/* It is a directory */
				{
					/* Do nothing */
				}
				else											/* It is a file. */
				{
					if (fileNumber < MAX_FILE_NUMBER)
					{
						char ** updatedFileList = (char**)realloc(fileList, (fileNumber + 1) * sizeof(char*));
						if (updatedFileList)
						{
							fileList = updatedFileList;
							fileList[fileNumber] = NULL;
						}
						else
							break;

						if (fno.lfname[0] == '\0')
							fileName = fno.fname;
						else
							fileName = fno.lfname;

						fileList[fileNumber] = strdup(fileName);
						if (fileList[fileNumber])
							fileNumber++;
						else
							break;
					}
					else
						break;
				}
			}
			f_closedir(&dir);
			free(fno.lfname);
		}
	}

    return res;
}

static FRESULT RemoveFileFromSD(char* path)
{
	FRESULT res;

	res = f_unlink(path);

	return res;
}

static void ClearFileList(void)
{
	while(fileNumber)
	{
		fileNumber--;
		free(fileList[fileNumber]);
	}
}

static inline uint8_t KeyPressed(BUTTON key, BUTTON oldKey)
{
	return ((key.state ^ oldKey.state) & oldKey.state);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
