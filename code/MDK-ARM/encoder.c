#include "encoder.h"
#include "delay.h"
#include "PWM.h"
int run_encoder(void)
{
    volatile int16_t encoder_cnt = 0, pre = 0, rate = 0;
    // Start the encoder
    TIM2->CR1 |= TIM_CR1_CEN;
	timer1_motor_init();

Timer1_PWM_duty_cycle(200,0);
	//Timer1_PWM_duty_cycle(0,1000);

    while (1)
    {
        delay_us(1000);
        encoder_cnt = TIM2->CNT;
        rate = encoder_cnt - pre;  // calculate the pulse rate per second
        pre = encoder_cnt;
    }
}

void encoder_init(void)
{
	    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOAEN;

    // Configure GPIOA pins for TIM2 CH1 and CH2
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1); // Alternate function mode
    GPIOA->AFR[0] |= 0x11; // Set AF1 (TIM2) for PA0 and PA1
	
    // Enable the clock for TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set the encoder mode
    TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // Encoder mode 3: Counter counts up/down on TI1FP1 and TI2FP2 edges

    // Set the input filter and polarity for TI1 and TI2
    TIM2->CCMR1 = (TIM_CCMR1_IC1F | TIM_CCMR1_IC2F) | (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

    // Set the auto-reload register to its maximum value
    TIM2->ARR = 0xFFFF;

    // Set the counter to zero
    TIM2->CNT = 0;
		
		    // Enable HSE (High Speed External) clock
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure the main PLL
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | 8 | (336 << 6) | (2 << 16) | (7 << 24);

    // Enable the main PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Configure Flash prefetch, Instruction cache, Data cache and wait state
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

    // Select the main PLL as system clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Configure the HCLK, PCLK1 and PCLK2 clocks dividers
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
}



