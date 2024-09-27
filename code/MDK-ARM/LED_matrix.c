#include "LED_matrix.h"
#include "delay.h"
#include <string.h>
unsigned char led[][8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t maquet[8] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};  
void LED_matrix_GPIO_Init(void) {
	// Enable clock for port B
    RCC->AHB1ENR |= (1U << 1); 
	
    GPIOB->MODER 	 &= ~	((3U << 13*2) | (3U << 15*2) | (3U << 2*2) ); 
    GPIOB->MODER 	 |= 	 (1U << 13*2) | (1U << 15*2) | (1U << 2*2) ; 
	
    GPIOB->OTYPER  &= ~	((1U << 13) 	| (1U << 15) 	 | (1U << 2));   // output push-pull
	
    GPIOB->OSPEEDR |= 	 (3U << 13*2) | (3U << 15*2) | (3U << 2*2) ; // HIGH SPEED
	
    GPIOB->PUPDR 	 &= ~	((1U << 13*2) | (1U << 15*2) | (1U << 2*2)); // NO pull-up/pull-down
}

void ShiftOut_m(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
			if ((data & (1 << (7-i))))  GPIOB->BSRR = (1U << 15); 		// Set DATA_PIN
        else 											GPIOB->BSRR = (1U <<(15+16));
        GPIOB->BSRR = (1U << 13); 						// Set CLK_PIN
        GPIOB->BSRR = (1U << (13+16)); 				// Reset CLK_PIN
    }
}

void display_matrix(uint8_t* rowData)
{
	for(int j=0;j<30;j++){
    for (int i = 0; i < 8; i++)
    {
				ShiftOut_m(maquet[i]);
        ShiftOut_m(rowData[7-i]);
				GPIOB->BSRR = (1U << 2);       LCD_delay_ms(1);  	// Set LATCH_PIN d? c?p nh?t hi?n th?
        GPIOB->BSRR = (1U << (2+ 16)); LCD_delay_ms(1);		// Reset LATCH_PIN d? hoàn thành c?p nh?t
		}
	}
}



void shift_left(uint8_t* data, int length)  {
    uint8_t temp = 0;
    for (int i = 0; i < length; i++) {
        temp = (data[i] << 1) ;
        data[i] = temp;
    }
}


void chaychu(uint8_t* data,int length) {
	 uint8_t buffer[8];
    memcpy(buffer, data, 8); // Copy original data to buffer

    for (int i = 0; i < 8; i++) {
        display_matrix(buffer);
			shift_left(buffer, length);
        
        LCD_delay_ms(1); // Adjust delay for desired speed
    }
}


