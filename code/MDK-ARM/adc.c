#include "adc.h"
#include "lcd.h"
#include "delay.h"
#include "GPIO.h"
#include <cstring>
#include <math.h>

uint16_t adc_values[5];
char vars[10], temp[10], hudminity[10]; 
char JOY_X[5], JOY_Y[5];    

void ADC_Init(void) {
    // Enable GPIOC clock
    RCC->AHB1ENR |= (1 << 2);

    // Enable ADC1 clock
    RCC->APB2ENR |= (1 << 8);

    // Configure PC0-PC4 as analog inputs
    GPIOC->MODER |= (3u << (2 * 0)) | (3u << (2 * 1)) | (3u << (2 * 2)) | (3u << (2 * 3)) | (3u << (2 * 4));

    ADC1->CR1 = 0;  // Clear CR1 register
    ADC1->CR2 = 0;  // Clear CR2 register

    // Set ADC common configuration
    ADC->CCR &= ~((3 << 0) | (3 << 1) | (3 << 2) | (3 << 3) | (3 << 4));  // Independent mode
    ADC->CCR |= (1 << 16) | (1 << 17);  // Prescaler 2
    ADC->CCR &= ~((3 << 14) | (3 << 15));  // DMA mode disabled
    ADC->CCR |= (1u << 11) | (1u << 10) | (1u << 9) | (1u << 8);  // Delay 20

    // Number of conversions
    ADC1->SQR1 |= (1 << 21);  // 3 conversions
    ADC1->SQR1 &= ~((1 << 23) | (1 << 22) | (1 << 20));  // 3 conversions

    // Configure ADC1 regular channels
    ADC1->SQR3 = (10 << 0) | (11 << 5) | (12 << 10);  // Channel 10, 11, 12

    // Sampling time configuration
    ADC1->SMPR1 = 0;

    // Start ADC1 Software Conversion
    ADC1->CR2 |= (1u << 30);

    // ADC configuration
    ADC1->CR2 |= (1 << 0);  // Enable ADC
    ADC1->CR2 |= (1 << 1);  // Continuous conversion mode

    for (int i = 0; i < 5; ++i) {
        adc_values[i] = ADC_Read(i + 10);  // Read channels 10 to 14
    }
}

void DMA_Init_Function(void) {
    // Enable DMA2 clock
    RCC->AHB1ENR |= (1 << 22);  // Enable DMA2 clock

    // Disable the stream
    DMA2_Stream0->CR &= ~(3<<0);

    // Wait for the EN bit to be cleared
    while (DMA2_Stream0->CR & (1<<0));

    // Clear all interrupt flags
    DMA2->LIFCR |= (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0);

    // Set the channel selection (Channel 0)
    DMA2_Stream0->CR &= ~(7 << 25);
    DMA2_Stream0->CR |= (0 << 25);  // Channel 0

    // Set the peripheral address
    DMA2_Stream0->PAR = (uint32_t)(&(ADC1->DR));

    // Set the memory address
    DMA2_Stream0->M0AR = (uint32_t)adc_values;

    // Set the number of data items to transfer
    DMA2_Stream0->NDTR = 3;

    // Configure the stream control register
    DMA2_Stream0->CR &= ~(3 << 6);  // Clear DIR bits
    DMA2_Stream0->CR |= (1 << 6);   // Peripheral to memory

    DMA2_Stream0->CR &= ~(3 << 13); // Clear MSIZE bits
    DMA2_Stream0->CR &= ~(3 << 11); // Clear PSIZE bits

    DMA2_Stream0->CR |= (1 << 10);  // Memory increment mode
    DMA2_Stream0->CR |= (1 << 8);   // Circular mode

    DMA2_Stream0->CR &= ~(3 << 16); // Clear PL bits
    DMA2_Stream0->CR |= (2 << 16);  // High priority

    // Configure FIFO control register
    DMA2_Stream0->FCR &= ~(1 << 2); // Direct mode disable
    DMA2_Stream0->FCR &= ~(3 << 0); // Clear FTH bits
    DMA2_Stream0->FCR |= (1 << 0);  // FIFO threshold half full

    // Enable the stream
    DMA2_Stream0->CR |= DMA_SxCR_EN;
}


uint16_t ADC_Read(uint8_t channel) {
    ADC1->SQR3 = channel;               // Select ADC channel
    ADC1->CR2 |= 1<<30;       // Start conversion
ADC1->CR2 |= (1<<0); // ADC1 enable
ADC1->CR2 |= (1<<30); //start
    while (!(ADC1->SR & (1<<1)));   // Wait for conversion complete

    return ADC1->DR;            // Return ADC converted value

}

void run_adc(int num){
	
	ADC_Init();
	//DMA_Init_Function();
        // Convert ADC values to meaningful data
				int var = adc_values[0] ;// * 3.3 / 4095.0;
        int nhietdo =-273.15 + (1.0 / ((log((4095.0 * 5.0 / ((int)adc_values[1] * 3.0) - 0.85) / 10.0) / 3435.0) + 1.0 / (25.0 + 273.15)));
        int doam = 100.0 * (int)adc_values[2] * 3.3 / (3.0 * 4095.0);
        if (doam >= 100.0) doam = 100.0;

        uint16_t joy_x = adc_values[3];
        uint16_t joy_y = adc_values[4];

        intToStr(var, vars);
        intToStr(nhietdo, temp);
        intToStr(doam, hudminity);
        intToStr(joy_x, JOY_X);
        intToStr(joy_y, JOY_Y);
				
				if(num==1){
				LCD_SetCursor(0, 0, b4);
        LCD_sendString("Nhietdo: ", b4);
        LCD_sendString(temp, b4);

        LCD_SetCursor(1, 0, b4);
        LCD_sendString("Doam: ", b4);
        LCD_sendString(hudminity, b4);
					}
				else if(num==2){
				
				LCD_SetCursor(0, 0, b4);
        LCD_sendString("Bientro: ", b4);
        LCD_sendString(vars, b4);
}
else{
				LCD_SetCursor(0, 0, b4);
        LCD_sendString("x: ", b4);
        LCD_sendString(JOY_X, b4);

        LCD_SetCursor(1, 0, b4);
        LCD_sendString("y: ", b4);
        LCD_sendString(JOY_Y, b4);
				}
					LCD_delay_ms(1000);
				lcd_clear(b4);
}
