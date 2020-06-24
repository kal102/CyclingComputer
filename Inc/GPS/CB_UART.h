/*
 * RingBuffer.h
 *
 * Created: 29.11.2017 20:28:30
 *  Author: Lukasz
 */ 


#ifndef CB_UART_H_
#define CB_UART_H_

#include <stdint.h>
#include <stdbool.h>
#define CB_UART_SIZE 600

enum bufferResult {BUFFER_FULL, BUFFER_OK};

typedef struct
{
	char elements[CB_UART_SIZE];
	uint16_t Begin;
	uint16_t Count;
} CB_UART;

_Bool CB_UART_IsEmpty(volatile CB_UART *cb);
_Bool CB_UART_IsFull(volatile CB_UART *cb);
enum bufferResult CB_UART_Add(volatile CB_UART *cb, char value);
char CB_UART_Read(volatile CB_UART *cb);

#endif /* CB_UART_H_ */
