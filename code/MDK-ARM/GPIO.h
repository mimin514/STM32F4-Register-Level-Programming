#ifndef GPIO_H
#define GPIO_H

#include "stm32f4xx.h"

void delay(void);
void LED_Configuration(void);
void LedBlink(void);
void LedAll(void);
int ReadButton(void);

#endif
