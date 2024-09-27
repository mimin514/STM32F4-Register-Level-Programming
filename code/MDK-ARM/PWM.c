#include "PWM.h"
#include "delay.h"


void timer4_init() {
    // Enable clock for Port D
    RCC->AHB1ENR |= (1 << 3);
    
    // Configure PD12 to PD15 as alternate function mode
    GPIOD->MODER &= ~((3U << (2*12)) | (3U << (2*13)) | (3U << (2*14)) | (3U << (2*15)));
    GPIOD->MODER |= ((2U << (2*12)) | (2U << (2*13)) | (2U << (2*14)) | (2U << (2*15)));
    
    // Configure PD12 to PD15 to AF2 (TIM4 AF)
    GPIOD->AFR[1] |= (2U << 16) | (2U << 20) | (2U << 24) | (2U << 28);
    
    // Enable clock for Timer 4
    RCC->APB1ENR |= (1 << 2);

    // Configure Timer 4 for PWM mode
    TIM4->CCER |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);
    TIM4->CR1 |= (1 << 7);
    TIM4->PSC = 83;  // Prescaler value to achieve 1 MHz frequency
    TIM4->ARR = 1000 - 1;  // Auto-reload register (period) to achieve 1 kHz PWM frequency

    TIM4->EGR |= (1 << 0);  // Update generation to load the prescaler value

    // Configure PWM mode 1 for channels 1 to 4
    TIM4->CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM4->CCMR2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    
    // Enable Timer 4
    TIM4->CR1 |= (1 << 0);
}

void timer1_motor_init(void) {
    // Enable clock for Port E
    RCC->AHB1ENR |= (1 << 4);
    
    // Configure PE9, PE11 as alternate function mode
    GPIOE->MODER &= ~((3 << (2*9)) | (3 << (2*11)));
    GPIOE->MODER |= ((2 << (2*9)) | (2 << (2*11)));
    
    // Configure PE9, PE11 to AF1 (TIM1 AF)
    GPIOE->AFR[1] |= (1 << 4) | (1 << 12);

    // Configure PE12 as general-purpose output for motor enable
    GPIOE->MODER &= ~(3 << (2*12));
    GPIOE->MODER |= (1 << (2*12));
    
    // Enable clock for Timer 1
    RCC->APB2ENR |= (1 << 0);

    // Configure Timer 1 for PWM mode
    TIM1->CCER |= (1 << 0) | (1 << 4);
    TIM1->CR1 |= (1 << 7);
    TIM1->PSC = 83;  // Prescaler value to achieve 1 MHz frequency
    TIM1->ARR = 1000 - 1;  // Auto-reload register (period) to achieve 1 kHz PWM frequency
    TIM1->EGR |= (1 << 0);  // Update generation to load the prescaler value

    // Configure PWM mode 1 for channels 1 and 2
    TIM1->CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);

    // Enable Timer 1
    TIM1->CR1 |= (1 << 0);
    TIM1->BDTR |= (1 << 15);  // Main output enable
}

void Timer4_PWM_duty_cycle(uint16_t Ch1, uint16_t Ch2, uint16_t Ch3 , uint16_t Ch4) {
    TIM4->CCR1 = Ch1; 
    TIM4->CCR2 = Ch2;
    TIM4->CCR3 = Ch3;
    TIM4->CCR4 = Ch4;
}
void Timer1_PWM_duty_cycle(uint16_t Ch1, uint16_t Ch2) {
    TIM1->CCR1 = Ch1;
    TIM1->CCR2 = Ch2;
}

void run_pwm(int num){
	if(num==1)
	for (uint16_t duty = 0; duty <=100; duty++) {
		Timer4_PWM_duty_cycle(duty, duty, duty, duty);
            LCD_delay_us(1000);
        }
//	Timer4_PWM_duty_cycle(100, 100, 100,100);
	else {
			 	for (uint16_t duty = 0; duty <=500; duty++) {
            Timer1_PWM_duty_cycle(500-duty, 500+duty);
					LCD_delay_us(70);
        }
//for (uint16_t duty = 0; duty <=300; duty++) {
            Timer1_PWM_duty_cycle(0, 0);
      //  }
		//LCD_delay_us(1000);
	}
}






//#include "stm32f4xx.h"

//void PWM_Init(void) {
//    // Enable GPIOA clock
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//    // Enable Timer 2 clock
//    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

//    // Set PA2 and PA3 to alternate function mode
//    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
//    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);

//    // Set alternate function type for PA2 and PA3 to AF1 (TIM2_CH3 and TIM2_CH4)
//    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
//    GPIOA->AFR[0] |= (GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL3_0);

//    // Set timer prescaler and auto-reload value for 1 kHz PWM frequency
//    TIM2->PSC = 1600 - 1; // Prescaler value
//    TIM2->ARR = 1000 - 1; // Auto-reload value

//    // Configure TIM2 channel 3 and channel 4 for PWM mode 1
//    TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
//    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);

//    // Enable preload register on TIM2 channel 3 and channel 4
//    TIM2->CCMR2 |= (TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);

//    // Set initial duty cycle (50%)
//    TIM2->CCR3 = 500; // 50% duty cycle
//    TIM2->CCR4 = 500; // 50% duty cycle

//    // Enable TIM2 channel 3 and channel 4 output
//    TIM2->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E);

//    // Enable auto-reload preload
//    TIM2->CR1 |= TIM_CR1_ARPE;

//    // Enable counter
//    TIM2->CR1 |= TIM_CR1_CEN;
//}

//int main(void) {
//    PWM_Init();
//    while (1) {
//        // Main loop
//    }
//}










//void timer1_Init(void) {
//    // Enable clocks for GPIOE and TIM1
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
//    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//    
//    // Configure PE9 and PE11 as alternate function (AF1 - TIM1)
//    GPIOE->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER11);
//    GPIOE->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER11_1);
//    
//    GPIOE->AFR[1] |= (0x01 << (4 * (9 - 8))) | (0x01 << (4 * (11 - 8)));
//    
//    // Configure PE12 as general purpose output for enable A
//    GPIOE->MODER &= ~GPIO_MODER_MODER12;
//    GPIOE->MODER |= GPIO_MODER_MODER12_0;

//    // Configure TIM1 for PWM mode
//    TIM1->PSC = 1600 - 1;          // Prescaler value for 10 kHz
//    TIM1->ARR = 1000 - 1;          // Auto-reload value for 10 kHz PWM frequency
//    TIM1->CCR1 = 500;              // Set duty cycle to 50%
//    TIM1->CCR2 = 500;              // Set duty cycle to 50%
//    
//    // Configure PWM mode 1 on channel 1 and channel 2
//    TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
//    TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
//    
//    // Enable capture/compare channels
//    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
//    
//    // Enable main output
//    TIM1->BDTR |= TIM_BDTR_MOE;
//    
//    // Enable counter
//    TIM1->CR1 |= TIM_CR1_CEN;
//		 GPIOE->ODR |= GPIO_ODR_OD12;
//}



//void Set_PWM_DutyCycle(uint8_t channel, uint16_t dutyCycle) {
//    if (dutyCycle > 1000) dutyCycle = 1000; // Ensure duty cycle is within bounds
//    switch (channel) {
//        case 1:
//            TIM1->CCR1 = dutyCycle;
//            break;
//        case 2:
//            TIM1->CCR2 = dutyCycle;
//            break;
//    }
//}
