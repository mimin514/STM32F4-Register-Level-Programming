#include "led.h"
#include "delay.h"

uint8_t ma7doan[] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};

//////////////////////////////////
// cau hinh chan //

void LED_7_GPIO_Init(void) {
	// Enable clock for port C, D
    RCC->AHB1ENR 	|= (1U << 2) | (1U << 3); 
	
    GPIOD->MODER 	&= ~((3U << 11*2) | (3U << 10*2) );
		GPIOC->MODER 	&= ~	(3U << 8*2); 
    GPIOD->MODER 	|= 	 (1U << 11*2) | (1U << 10*2) ;
		GPIOC->MODER 	|= 		(1U << 8*2); 
	
    GPIOD->OTYPER  &= ~((1U << 11) 	| (1U << 10) )	;//  output push-pull
		GPIOC->OTYPER  &= ~	(1U << 8); 
	
    GPIOD->OSPEEDR |=  (3U << 11*2) | (3U << 10*2)  ; // HIGH SPEED
		GPIOC->OSPEEDR |=  (3U << 8*2);
	
    GPIOD->PUPDR 	&= ~((1U << 11*2) | (1U << 10*2)) ; // NO pull-up/pull-down
		GPIOC->PUPDR 	&= ~ (1U << 8*2);
}

//////////////////////////////////
// Gui 1 byte toi shift register //
void shiftOut_7(uint8_t data) {
	unsigned int tam, i;
    tam = data;
    for ( i = 0; i < 8; i++) {
        if ((tam&0x80) == 0x80)  	{GPIOD->BSRR = (1U << 10); 		}// Set DATA_PIN
        else 											{GPIOD->BSRR = (1U <<(10+16)); }// Reset DATA_PIN
        GPIOD->BSRR = (1U << 11);					 	//LCD_delay_ms(1);// Set CLK_PIN
        GPIOD->BSRR = (1U << (11+16)); 			// Reset CLK_PIN
			tam <<= 1;
			
    }
}


//////////////////////////////
// hien thi
void selectDigit(uint8_t digit){
uint8_t positions[4] = {0x01, 0x02, 0x04, 0x08}; // Chân 1234
    for (uint8_t i = 0; i < 4; i++) {
       if(i==digit)
        shiftOut_7(positions[i]);
			 
    }
}
void selectNum(uint8_t num) {
		if(num >9) return;
    shiftOut_7(ma7doan[num]);
		GPIOC->BSRR = (1U << 8);        LCD_delay_ms(100);  // Set LATCH_PIN 
    GPIOC->BSRR = (1U << (8 + 16)); LCD_delay_ms(100);// Reset LATCH_PIN 
    
}

void display__7(unsigned int n) {
    uint8_t a, b, c, d;
    a = n / 1000;
    b = (n / 100) % 10;
    c = (n / 10) % 10;
    d = n % 10;
	
    uint8_t digits[4] = {a, b, c, d};
    uint8_t positions[4] = {0x02, 0x04, 0x08,0x01}; // Chân 1234
		
for(int j=0;j<1000;j++){
    for (uint8_t i = 0; i < 4; i++) {
        shiftOut_7(ma7doan[digits[i]]);
			
				GPIOC->BSRR = (1U << 8);      	//LCD_delay_us(50);   // Set LATCH_PIN 
        GPIOC->BSRR = (1U << (8 + 16));	//LCD_delay_us(1); // Reset LATCH_PIN 
			  shiftOut_7(positions[i]);
			
			//LCD_delay_us(5);
		}// 
		}
		
}

