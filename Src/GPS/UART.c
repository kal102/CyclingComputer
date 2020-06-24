#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l1xx_hal.h"
#include "GPS/UART.h"
#include <GPS/CB_UART.h>

volatile uint8_t recByte;
volatile CB_UART recBuff= { .Begin = 0, .Count = 0};
volatile uint8_t msgCounter = 0;
volatile uint8_t TxFlag = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (recByte == '\r')
	{
		recByte = '\0';
		CB_UART_Add(&recBuff, recByte);
		++msgCounter;
	}
	else if (recByte != '\n')
	{
		CB_UART_Add(&recBuff, recByte);
	}

	HAL_UART_Receive_IT(&huart1, &recByte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	TxFlag = 0;
}

_Bool GetToken(char msg[MAX_MSG_LEN])
{
	char character;

	for (uint8_t i = 0; i < MAX_MSG_LEN; i++)
	{
		if (CB_UART_IsEmpty(&recBuff))
		{
			return false;
		}
		character = CB_UART_Read(&recBuff);
		msg[i] = character;
		if (character == '\0')
		{
			break;
		}
	}
	return true;
}

void UART_startReceiving(void)
{
	HAL_UART_Receive_IT(&huart1, &recByte, 1);
}

uint8_t UART_getMsgCounter(void)
{
	return msgCounter;
}

uint8_t UART_getTxFlag(void)
{
	return TxFlag;
}

void UART_getMessage(char msg[MAX_MSG_LEN])
{
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	msgCounter--;
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	GetToken(msg);
}

void UART_sendByte(uint8_t byte)
{
	TxFlag = 1;
	HAL_UART_Transmit_IT(&huart1, &byte, 1);
}

void UART_sendArray(uint8_t *array, uint8_t arraySize)
{
	TxFlag = 1;
	HAL_UART_Transmit_IT(&huart1, array, arraySize);
}

void UART_sendString(char *txt)
{
	TxFlag = 1;
	HAL_UART_Transmit_IT(&huart1, txt, strlen(txt));
}
