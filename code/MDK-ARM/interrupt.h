#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include "stm32f4xx.h"
#include "interrupt.h"
#include "delay.h"

void TIM2_Configuration(void) ;
void GPIO_Configuration(void) ;

void UART2_Configuration(void) ;
void TIM2_IRQHandler(void) ;
void USART2_IRQHandler(void);
void EXTI0_Configuration(void);

#endif
