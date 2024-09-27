#ifndef LED_H
#define LED_H

#include "stm32f4xx.h"
	
// cau hinh chan
void LED_7_GPIO_Init(void);

// hien thi

void selectDigit(uint8_t digit); //chon vi tri
void selectNum(uint8_t num);		 //chon so (1 bit)
void display(unsigned int n) ;
void display__7(unsigned int n); // chon so (4 bit)

#endif // LED7SEGMENT_H
