/*
 * UART.h
 *
 * Created: 29.11.2017 21:02:38
 *  Author: Lukasz
 */ 


#ifndef UART_H_
#define UART_H_

#include <GPS/CB_UART.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32l1xx_hal.h>

#define MAX_MSG_LEN 100

extern UART_HandleTypeDef huart1;

void UART_startReceiving(void);
void UART_getMessage(char msg[MAX_MSG_LEN]);
uint8_t UART_getMsgCounter(void);
void UART_sendByte(uint8_t byte);
void UART_sendArray(uint8_t *array, uint8_t arraySize);
void UART_sendString(char *txt);
uint8_t UART_getTxFlag(void);

#endif /* UART_H_ */
