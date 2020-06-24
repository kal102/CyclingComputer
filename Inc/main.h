/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx_ll_spi.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_cortex.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_dma.h"

#include "stm32l1xx_ll_exti.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BATTERY_ADC_Pin GPIO_PIN_0
#define BATTERY_ADC_GPIO_Port GPIOC
#define BUTTON_OK_Pin GPIO_PIN_0
#define BUTTON_OK_GPIO_Port GPIOA
#define BUTTON_OK_EXTI_IRQn EXTI0_IRQn
#define BUTTON_CANCEL_Pin GPIO_PIN_1
#define BUTTON_CANCEL_GPIO_Port GPIOA
#define BUTTON_LEFT_Pin GPIO_PIN_2
#define BUTTON_LEFT_GPIO_Port GPIOA
#define BUTTON_RIGHT_Pin GPIO_PIN_3
#define BUTTON_RIGHT_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_12
#define OLED_CS_GPIO_Port GPIOB
#define OLED_CLK_Pin GPIO_PIN_13
#define OLED_CLK_GPIO_Port GPIOB
#define OLED_MOSI_Pin GPIO_PIN_15
#define OLED_MOSI_GPIO_Port GPIOB
#define CPU_LOAD_Pin GPIO_PIN_7
#define CPU_LOAD_GPIO_Port GPIOC
#define SD_CD_Pin GPIO_PIN_8
#define SD_CD_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_9
#define SD_CS_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define SD_SCLK_Pin GPIO_PIN_10
#define SD_SCLK_GPIO_Port GPIOC
#define SD_MISO_Pin GPIO_PIN_11
#define SD_MISO_GPIO_Port GPIOC
#define SD_MOSI_Pin GPIO_PIN_12
#define SD_MOSI_GPIO_Port GPIOC
#define BMP280_SCL_Pin GPIO_PIN_6
#define BMP280_SCL_GPIO_Port GPIOB
#define BMP280_SDA_Pin GPIO_PIN_7
#define BMP280_SDA_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define TICKS_PER_SEC 100
#define MEASUREMENT_PERIOD 50
#define DISPLAY_PERIOD 20
#define ADC_MAX_VALUE 4096
#define SUPPLY_VOLTAGE 300
#define BATTERY_VOLTAGE_MIN 330
#define BATTERY_VOLTAGE_MAX 420
#define BATTERY_LEVEL_LOW 20
#define STOP_MODE_TICKS 200
#define STOP_MODE_MINUTES 10
#define INFINITE 0xFFFFFFFF
#define MAX_FILE_NUMBER 99
#define DELETE_FILE_TICKS 100

enum BatteryState {BATTERY_OK, BATTERY_LOW, BATTERY_DISCHARGED, BATTERY_ERROR};
enum NotificationType {NONE, WELCOME, START, STOP, PAUSE, AUTOPAUSE, WEAK_GPS_SIGNAL, WAITING_FOR_GPS, SD_CARD_ERROR, NO_FILES_FOUND};

typedef enum {HILI_NONE,HILI_VALUE,HILI_VALUE_SQUARE,HILI_INDEX} HIGHLIGHT;

typedef struct
{
	uint8_t level;
	enum BatteryState state;
} BATTERY;

typedef struct
{
	uint8_t state;
	uint16_t holdCount;
} BUTTON;

typedef struct
{
	BUTTON ok;
	BUTTON cancel;
	BUTTON left;
	BUTTON right;
} BUTTONS;

typedef struct
{
	uint8_t menu;
	uint8_t screen;
} MENU;

typedef struct
{
	enum NotificationType type;
	uint32_t duration;
} NOTIFICATION;

typedef struct
{
	uint8_t x;
	uint8_t y;
} CURSOR;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
