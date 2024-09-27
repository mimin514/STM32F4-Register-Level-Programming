#include "GPIO.h"
#include "delay.h"
void delay(void){
	for (volatile uint32_t j = 0; j < 4000000; ++j);
}
// Configure LEDs connected to PD12, PD13, PD14, PD15
void LED_Configuration(void) {
    // Enable clock for GPIOD
    RCC->AHB1ENR |=(1U<<3);
    // Configure PD12, PD13, PD14, PD15 as output
		GPIOD->MODER |= 	((1U<<24) | (1U<<26) | (1U<<28) | (1U<<30));
    // Configure PD12, PD13, PD14, PD15 as push-pull
    GPIOD->OTYPER &= ~((1U<<12) | (1U<<13) | (1U<<14) | (1U<<15)); 
    // Configure PD12, PD13, PD14, PD15 as very high speed
    GPIOD->OSPEEDR |= ((3U<<24) | (3U<<26) | (3U<<28)| (3U<<30)); 
}
// Flash all LEDs (PD12, PD13, PD14, PD15) on and off sequentially
void LedBlink(void) {
    GPIOD->BSRR |=(1U<<12); // Turn on PD12
    delay_ms(30); // Delay
    GPIOD->BSRR |=(1U<<28); // Turn off PD12
    GPIOD->BSRR |=(1U<<13); // Turn on PD13
    delay_ms(30); // Delay
    GPIOD->BSRR |=(1U<<29); // Turn off PD13
    GPIOD->BSRR |=(1U<<14); // Turn on PD14
    delay_ms(30); // Delay
    GPIOD->BSRR |=(1U<<30); // Turn off PD14
    GPIOD->BSRR |=(1U<<15); // Turn on PD15
    delay_ms(30); // Delay
    GPIOD->BSRR |=(1U<<31); // Turn off PD15
}
// Sequentially flash LEDs from PD12 to PD15
void LedAll(void) {
    GPIOD->BSRR |=(1U<<12); // Turn on PD12
    GPIOD->BSRR |=(1U<<13); // Turn on PD13
		GPIOD->BSRR |=(1U<<14); // Turn on PD14
		GPIOD->BSRR |=(1U<<15); // Turn on PD15
		delay_ms(5000); // Delay
    GPIOD->BSRR |=(1U<<28); // Turn off PD12
    GPIOD->BSRR |=(1U<<29); // Turn off PD13
    GPIOD->BSRR |=(1U<<30); // Turn off PD14
    GPIOD->BSRR |=(1U<<31); // Turn off PD15
	delay_ms(5000); 
}
// Read the state of the button connected to PA0
int ReadButton(void) {
    // Enable clock for GPIOA
    RCC->AHB1ENR |= (1U<<0);
    // Configure PA0 as input with pull-up
    GPIOA->MODER &= ~(3U<<0); // Input mode
    GPIOA->PUPDR |= (1U<<0);// Pull-up
    // Return the state of PA0
    return (GPIOA->IDR & (1U << 0)) ? 1 : 0; // 1 if pressed, 0 otherwise
}
