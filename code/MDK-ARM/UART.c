#include "UART.h"
#include "delay.h"

void GPIO_Init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= 1<<0;

    // Configure PA9 (TX) and PA10 (RX) as alternate function
    GPIOA->MODER &= ~(1<<9*2 | 1<<10*2);
    GPIOA->MODER |= (1<<9*2 | 1<<10*2);

    // Set alternate function to AF7 (USART1)
    GPIOA->AFR[1] |= (0x7 << (4 * 1)) | (0x7 << (4 * 2)); // PA9 (AF7), PA10 (AF7)

    // Configure PA9 and PA10 as high speed
    GPIOA->OSPEEDR |= (1<<9*2 | 1<<10*2);
	
	// Enable HSE (High Speed External) clock
    RCC->CR |= 1<<16;
    while (!(RCC->CR & 1<<17));

    // Configure the main PLL
    RCC->PLLCFGR = 1<<22 | (8 << 0) |
                   (336 << 6) |      // PLLN starting at bit 6
               (0 << 16) |       // PLLP = 2 -> 00 starting at bit 16
               (7 << 24);

    // Enable the main PLL
    RCC->CR |= 1<<24;
    while (!(RCC->CR & 1<<25));

    // Configure Flash prefetch, Instruction cache, Data cache, and wait state
    FLASH->ACR = 1<<9 | 1<<10 | 1<<2|1<<0;

    // Select the main PLL as system clock source
    RCC->CFGR |= 2<<0;
    while ((RCC->CFGR & 1<<2 & 1<<3) != 2<<2);

    // Configure the HCLK, PCLK1, and PCLK2 clocks dividers
    RCC->CFGR |= 1<<3;
    RCC->CFGR |= 1<<12|1<<10;
    RCC->CFGR |= 1<<15;
}

void UART_Init(uint32_t baudrate) {
    // Enable USART1 clock
    RCC->APB2ENR |= 1<<4;

    // Configure baud rate
    //USART1->BRR = SystemCoreClock / baudrate;
USART1->BRR &=~(1<<12);
USART1->BRR |= ((int)546<<4)|(14<<0);
   // USART1->BRR = 16800000 / 9600;
    // Enable USART, transmitter, and receiver
    USART1->CR1 |= 1<<13 |1<<3 | 1<<2;
}

void transmit_data(uint8_t data) {
    // Wait until TXE (Transmit data register empty) is set
    while (!(USART1->SR & 1<<7));
    // Send data
    USART1->DR = data;

}

uint8_t receive_data(void) {
    // Wait until RXNE (Read data register not empty) is set
    while (!(USART1->SR & 1<<5));
    // Read data
    return (char)(USART1->DR & 0xFF);
}
void Send_string_uart(char* str) {
    while (*str) {
        transmit_data((uint8_t)*str++);
    }
}
