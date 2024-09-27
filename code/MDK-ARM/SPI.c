#include "stm32f4xx.h"


void SPI1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;   // Enable SPI1 clock

    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);  // Set to Alternate Function mode

    GPIOA->AFR[0] |= (5 << 20) | (5 << 24) | (5 << 28);  // Set AF5 for SPI1

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0; // Baud rate: f_PCLK/4
    SPI1->CR1 &= ~SPI_CR1_CPOL; // Clock Polarity: 0 when idle
    SPI1->CR1 &= ~SPI_CR1_CPHA; // Clock Phase: 1st edge
    SPI1->CR2 = 0x0000;         // No specific configuration needed

    SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI1
}

void SPI_SendData(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));  // Wait until TX buffer is empty
    SPI1->DR = data;                   // Send the data
    while (SPI1->SR & SPI_SR_BSY);     // Wait until SPI is not busy
}

void MAX7219_Send(uint8_t address, uint8_t data) {
    GPIOA->BSRR = GPIO_BSRR_BR_4;  // Set CS low
    SPI_SendData(address);         // Send address
    SPI_SendData(data);            // Send data
    GPIOA->BSRR = GPIO_BSRR_BS_4;  // Set CS high
}
void MAX7219_DisplayDigit(uint8_t digit, uint8_t value) {
    if (digit >= 1 && digit <= 8) {
        MAX7219_Send(digit, value);
    }
}

void DisplayNumber(uint32_t number) {
    for (int i =1; i <= 8; i++) { // Display from right to left
        uint8_t digit = number % 10;
        MAX7219_DisplayDigit(i, digit);
        number /= 10;
    }
}

void USART2_Init(void) {
    // 1. Enable the clock for GPIOA and USART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 clock

    // 2. Configure PA2 (TX) as Alternate Function for USART2
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);  // Clear MODER for PA2
    GPIOA->MODER |= (GPIO_MODER_MODER2_1); // Set PA2 to Alternate Function mode

    // 3. Configure PA2 for AF7 (USART2)
    GPIOA->AFR[0] |= 0x0700;  // Set AF7 (USART2) for PA2

    // 4. Configure USART2
    USART2->BRR = 0x0683;      // Baud rate 9600 @16MHz
    USART2->CR1 |= USART_CR1_TE;  // Enable Transmitter
    USART2->CR1 |= USART_CR1_UE;  // Enable USART2
}

void USART2_SendData(uint8_t data) {
    // Wait until TXE (Transmit Data Register Empty) is set
    while (!(USART2->SR & USART_SR_TXE));

    // Send data
    USART2->DR = data;

    // Wait until the transmission is complete
    while (!(USART2->SR & USART_SR_TC));
}


void GPIO_Init_spi(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock
    GPIOA->MODER &= ~(GPIO_MODER_MODER4); // Clear MODER for PA4
    GPIOA->MODER |= (GPIO_MODER_MODER4_0); // Set PA4 to output mode
    GPIOA->BSRR = GPIO_BSRR_BS_4; // Set CS high initially
}

void MAX7219_Init(void) {
    MAX7219_Send(0x09, 0xFF);  // Decode mode: decode for digits 0-7
    MAX7219_Send(0x0A, 0x0F);  // Intensity: max (0x00 to 0x0F)
    MAX7219_Send(0x0B, 0x07);  // Scan limit: display digits 0-7
    MAX7219_Send(0x0C, 0x01);  // Shutdown register: normal operation
    MAX7219_Send(0x0F, 0x00);  // Display test: off
}

