/******************************************************************************
**************************Hardware interface layer*****************************
* | file      	:	DEV_Config.h
* |	version		:	V1.0
* | date		:	2017-08-14
* | function	:	
	Provide the hardware underlying interface	
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_gpio.h"
#include "main.h"

#define USE_SPI_4W 1
#define USE_IIC 0

#define IIC_CMD        0X00
#define IIC_RAM        0X40

//OLED GPIO

#define OLED_CS_GPIO_Port	GPIOB
#define OLED_DC_GPIO_Port	GPIOB
#define OLED_RST_GPIO_Port	GPIOB

#define OLED_CS_Pin		GPIO_PIN_12
#define OLED_DC_Pin		GPIO_PIN_11
#define OLED_RST_Pin	GPIO_PIN_10

#define OLED_CS_0		HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
#define OLED_CS_1		HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)

#define OLED_DC_0		HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
#define OLED_DC_1		HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)

#define OLED_RST_0		HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)
#define OLED_RST_1		HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)

/*------------------------------------------------------------------------------------------------------*/

uint8_t System_Init(void);
void    System_Exit(void);

void SPI4W_Write_Byte(uint8_t value);

void Driver_Delay_ms(uint32_t xms);
void Driver_Delay_us(uint32_t xus);

#endif
