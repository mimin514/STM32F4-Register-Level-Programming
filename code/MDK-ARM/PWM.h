#ifndef PWM_H
#define PWM_H

#include "stm32f4xx.h"

void timer4_init(void);
void timer1_motor_init(void);
void run_pwm(int num);
void timer1_Init(void);
void Timer1_PWM_duty_cycle(uint16_t Ch1, uint16_t Ch2);
#endif

