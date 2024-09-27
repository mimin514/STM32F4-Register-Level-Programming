#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"

void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);
void Convert_Float_To_String(float value, char *str, uint8_t precision);
void run_adc(int num); // num: 1 for nhiet do do am, 2 for var, 3 for x y
#endif /* LCD_H_ */



