#include "interrupt.h"
#include "delay.h"
#include "GPIO.h"
#include "PWM.h"
volatile uint8_t led_off = 0;

void GPIO_Configuration(void) {
    // B?t d?ng h? cho GPIO D v� A
    RCC->AHB1ENR |= 1<<0 | 1<<3;

    // C?u h�nh PD12 l� ch�n d?u ra
    GPIOD->MODER &= ~((3U << (12 * 2))|(3U << (13 * 2))|(3U << (14 * 2))|(3U << (15 * 2))); // X�a bit 12
    GPIOD->MODER |= (1U << (12 * 2))|(1U << (13 * 2))|(1U << (14 * 2))|(1U << (15 * 2));  // �?t bit 12 th�nh 01 (Output mode)
    GPIOD->OTYPER &= ~((1U << 12)|(1U << 13)|(1U << 14)|(1U << 15));      // Push-pull
    GPIOD->OSPEEDR |=((3U << (12 * 2))|(3U << (13 * 2))|(3U << (14 * 2))|(3U << (15 * 2)));// C?u h�nh t?c d? cao cho PD12

    // C?u h�nh PA0 l� ch�n d?u v�o
    GPIOA->MODER &= ~(3U << (0 * 2)); // Ch? d? m?c d?nh (Input mode)
    GPIOA->PUPDR |= (1U << (0 * 2));  // K�o l�n (Pull-up)
}

// C?u h�nh Timer2
void TIM2_Configuration(void) {
    // B?t d?ng h? cho Timer2
    RCC->APB1ENR |= 1<<0;

    // C?u h�nh Timer2
    TIM2->PSC = 15999; // Prescaler: 16MHz / 16000 = 1kHz (1ms per tick)
    TIM2->ARR = 999;   // Auto-reload value: 1000 ticks = 1 gi�y
    TIM2->DIER |= 1<<0; // K�ch ho?t ng?t c?p nh?t
    TIM2->CR1 |= 1<<0;   // B?t Timer2

    // C?u h�nh NVIC d? k�ch ho?t ng?t Timer2
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // C?u h�nh nh�m uu ti�n
    NVIC_SetPriority(TIM2_IRQn, 0); // Uu ti�n ng?t cao nh?t
    NVIC_EnableIRQ(TIM2_IRQn); // K�ch ho?t ng?t Timer2
}


// H�m x? l� ng?t Timer2
void TIM2_IRQHandler(void) {
    if (TIM2->SR & 1<<0) { // Ki?m tra c? ng?t c?p nh?t
        TIM2->SR &= ~1<<0; // X�a c? ng?t
        GPIOD->ODR ^= (1U<<12)| (1U << 13) ; // �?o tr?ng th�i LED PD13 - PD15
			delay_us(1);
    }
}

void EXTI0_Configuration(void) {
    // B?t d?ng h? cho SYSCFG
    RCC->APB2ENR |= 1<<14;

    // C?u h�nh ng?t cho PA0
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0); // X�a c?u h�nh hi?n t?i
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Ch?n PA0 l�m ngu?n ng?t

    EXTI->IMR |= 1<<0; // B?t ng?t d�ng 0
    EXTI->RTSR |= 1<<0; // K�ch ho?t ng?t c?nh l�n
    EXTI->FTSR |= 1<<0; // K�ch ho?t ng?t c?nh xu?ng

    // C?u h�nh NVIC d? k�ch ho?t ng?t EXTI0
    NVIC_SetPriority(EXTI0_IRQn, 1); // Uu ti�n ng?t th?p hon Timer2 v� UART2
    NVIC_EnableIRQ(EXTI0_IRQn); // K�ch ho?t ng?t EXTI0
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & 1<<0) { // Ki?m tra c? ng?t d�ng 0
        EXTI->PR |= 1<<0; // X�a c? ng?t
if (GPIOA->IDR & (1U << 0))
		GPIOD->ODR &=~ ((1U << 14) | (1U << 15));     
		else {
			GPIOD->ODR |= ((1U << 14) | (1U << 15));
			delay_us(10);}
		}	
    }

void UART2_Configuration(void) {
    // B?t d?ng h? cho UART2 v� GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // C?u h�nh PA2 l� ch�n TX
    GPIOA->MODER |= (2U << (2 * 2));  // Alternate function mode
    GPIOA->AFR[0] |= (7U << (2 * 4)); // AF7 (UART2 TX)

    // C?u h�nh PA3 l� ch�n RX
    GPIOA->MODER |= (2U << (3 * 2));  // Alternate function mode
    GPIOA->AFR[0] |= (7U << (3 * 4)); // AF7 (UART2 RX)

    // C?u h�nh UART2
    USART2->BRR = 0x683; // Baud rate 9600 v?i 16MHz PCLK1
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Enable TX, RX, and RX interrupt
    USART2->CR1 |= USART_CR1_UE; // B?t UART2

    // C?u h�nh NVIC d? k�ch ho?t ng?t UART2
    NVIC_SetPriority(USART2_IRQn, 2); // Uu ti�n ng?t th?p hon Timer2
    NVIC_EnableIRQ(USART2_IRQn); // K�ch ho?t ng?t UART2
	timer1_motor_init();
}
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) { // Ki?m tra c? ng?t nh?n d? li?u
        char received = USART2->DR; // �?c d? li?u t? UART2

        if (received == 'L') { // N?u nh?n k� t? 'L', b?t LED
            Timer1_PWM_duty_cycle(1000,0);
        } else if (received == 'O') { // N?u nh?n k� t? 'O', t?t LED
            Timer1_PWM_duty_cycle(0,1000);
        }
    }
}
