#ifndef SPI_H
#define SPI_H

#include "stm32f4xx.h"
void SPI1_Init(void);
void SPI_SendData(uint8_t data);
void MAX7219_Send(uint8_t address, uint8_t data) ;
void DisplayNumber(uint32_t number) ;
void USART2_Init(void);
void USART2_SendData(uint8_t data);
void GPIO_Init_spi(void);
void MAX7219_Init(void);
void MAX7219_DisplayDigit(uint8_t digit, uint8_t value);

#endif // SPI_MAX7219_H
