#ifndef UART_H_
#define UART_H_

#include "stm32f4xx.h"

#include "UART.h"
#include "delay.h"

void GPIO_Init(void) ;

void UART_Init(uint32_t baudrate) ;

void transmit_data(uint8_t data) ;

uint8_t receive_data(void);
void Send_string_uart(char* str);

#endif

