#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include "stm32f4xx.h"

void LED_matrix_GPIO_Init(void);

void display_matrix(uint8_t* rowData);

void chaychu(uint8_t* data,int length);


#endif // LED7SEGMENT_H
