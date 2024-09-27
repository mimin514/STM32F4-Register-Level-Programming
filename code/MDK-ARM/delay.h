#ifndef DELAY_H
#define DELAY_H

#include "stm32f4xx.h" 

//void delay_init();
void delay_init(void) ;
void delay_ms(uint16_t ms);
void LCD_delay_us(uint16_t us) ;
void LCD_delay_ms(uint16_t ms) ;
void delay_us(uint16_t us);
#endif

