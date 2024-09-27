#include "delay.h"
void SysTick_Init(void) {
    SystemCoreClockUpdate();  // C?p nh?t l?i t?n s? h? th?ng
    SysTick_Config(168000000 / 1000);  // Thi?t l?p SysTick ch?y v?i chu k? 1ms
}
void delay_init(void) {
RCC->APB1ENR |= (1<<0); // enable clock for TIM2 

	TIM2->ARR = 0xffff-1;  	// ARR value
	TIM2->PSC = 72-1;      	// Prescalar value
	
	TIM2->CR1 |= (1<<0);  	// enable timer
	while (!(TIM2->SR & (1<<0)));
}

void LCD_delay_us(uint16_t us) {
    for(int i = 0; i < us*400; i++) {
        __NOP();
    };
}
void LCD_delay_ms(uint16_t ms) {
    for(int i = 0; i < ms*4000; i++) {
        __NOP();
    };
}
volatile int i;
void delay_ms(uint16_t ms){
	for (i=0; i<ms; i++)
	{
		delay_us(1000); // delay of 1 ms
	}
}

void delay_us(uint16_t us){
	TIM2->CNT = 0;
	while (TIM2->CNT < us);
}

















