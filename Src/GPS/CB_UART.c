/*
 * RingBuffer.c
 *
 * Created: 29.11.2017 20:31:15
 *  Author: Lukasz
 */ 

#include <stdint.h>
#include <stdbool.h>
#include "stm32l1xx_hal.h"
#include <GPS/CB_UART.h>

inline _Bool CB_UART_IsEmpty(volatile CB_UART *cb)
{
	return (cb->Count == 0);
}

inline _Bool CB_UART_IsFull(volatile CB_UART *cb)
{
	return (cb->Count == CB_UART_SIZE);
}

enum bufferResult CB_UART_Add(volatile CB_UART *cb, char value)
{
	uint16_t cb_end;

	if (CB_UART_IsFull(cb)) return BUFFER_FULL;
	cb_end = (cb->Begin + cb->Count) % CB_UART_SIZE;
	cb->elements[cb_end] = value;
	(cb->Count)++;

	return BUFFER_OK;
}

char CB_UART_Read(volatile CB_UART *cb)
{
	char value;

	if (CB_UART_IsEmpty(cb)) return '\0';
	value = cb->elements[cb->Begin];
	cb->Begin = (cb->Begin + 1) % CB_UART_SIZE;
	(cb->Count)--;

	return value;
}


