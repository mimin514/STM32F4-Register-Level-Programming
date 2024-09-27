#include "LCD.h"
#include "delay.h"

 void LCD_GPIO_Init(void) // cau hinh chan
{
    // Enable clock for GPIOA : RS(PA8)
    RCC->AHB1ENR |= (1U << 0) ; 
    GPIOA->MODER &= ~((3U << 8*2)|(3U << 15*2)); // Clear mode for PA8
    GPIOA->MODER |= ((1U << 8*2)|(1U << 15*2));  // Set PA8 as output

    // Enable clock for GPIOB : D3(PB3), D6(PB4), D5(PB5), EN(PB9)
    RCC->AHB1ENR |= (1U << 1); 
    GPIOB->MODER &= ~( (3U << 6) | (3U << 8) | (3U << 10) | (3U << 18) ); // Clear mode for PB3, PB4, PB5, PB9
    GPIOB->MODER |= (1U << 6) | (1U << 8) | (1U << 10) | (1U << 18);  // Set PB3, PB4, PB5, PB9 as output

    // Enable clock for GPIOC : D0(PC12), D1(PC10), D2(PC11)
    RCC->AHB1ENR |= (1U << 2); 
    GPIOC->MODER &= ~( (3U << 20) | (3U << 22) | (3U << 24) ); // Clear mode for PC10, PC11, PC12
    GPIOC->MODER |= (1U << 20) | (1U << 22) | (1U << 24);  // Set PC10, PC11, PC12 as output

    // Enable clock for GPIOE : D4(PE0), D7(PE1)
    RCC->AHB1ENR |= (1U << 4); 
    GPIOE->MODER &= ~( (3U << 0) | (3U << 2) ); // Clear mode for PE0, PE1
    GPIOE->MODER |= (1U << 0) | (1U << 2);  // Set PE0, PE1 as output
}


void LCD_send_Bit(char D, LCD_Mode mode) {
    GPIOC->ODR &= ~((1U << 10) | (1U << 11) | (1U << 12)); // Clear PC10, PC11, PC12
    GPIOB->ODR &= ~((1U << 3) | (1U << 4) | (1U << 5)); // Clear PB3, PB4, PB5
    GPIOE->ODR &= ~((1U << 0) | (1U << 1)); // Clear PE0, PE1
    
	if (mode == b8) {// Set 8 bits according to D
    if (D & 0x01) GPIOC->ODR |= (1U << 12); // D0
    if (D & 0x02) GPIOC->ODR |= (1U << 10); // D1
    if (D & 0x04) GPIOC->ODR |= (1U << 11); // D2
    if (D & 0x08) GPIOB->ODR |= (1U << 3);}  // D3
		// Set 4 bits according to D
    if (D & 0x10) GPIOE->ODR |= (1U << 0);  // D4
    if (D & 0x20) GPIOB->ODR |= (1U << 5);  // D5
    if (D & 0x40) GPIOB->ODR |= (1U << 4);  // D6
    if (D & 0x80) GPIOE->ODR |= (1U << 1);  // D7
	}

void LCD_sendCMD(char cmd, LCD_Mode mode) {
		GPIOA->ODR &= ~(1U << 8); // RS=0 : ghi lenh
	
    if (mode == b8) {
			LCD_send_Bit(cmd, b8); 
			GPIOB->ODR |= (1U << 9);  // EN=1
			LCD_delay_us(40);
			GPIOB->ODR &= ~(1U << 9); // EN=0
			LCD_delay_us(40);
		}		
		else { // Send higher nibble
			LCD_send_Bit(cmd& 0xF0, b4); 
			GPIOB->ODR |= (1U << 9);  // EN=1
			LCD_delay_us(40);
			GPIOB->ODR &= ~(1U << 9); // EN=0
			LCD_delay_us(40);
			// Send lower nibble
			LCD_send_Bit(cmd <<4, b4); 
			GPIOB->ODR |= (1U << 9);  // EN=1
			LCD_delay_us(40);
			GPIOB->ODR &= ~(1U << 9); // EN=0
			LCD_delay_us(40);
		}
}

void LCD_sendChar(char Char, LCD_Mode mode) {
		GPIOA->ODR |= (1U << 8); // RS=1 : ghi du lieu
	
	if (mode == b8) {
			LCD_send_Bit(Char, mode); 
			GPIOB->ODR |= (1U << 9); // EN=1
			LCD_delay_us(40);
			GPIOB->ODR &= ~(1U << 9);//EN=0
			LCD_delay_us(40);
		}
	else 
		{ // Send higher nibble
			LCD_send_Bit(Char& 0xF0, mode); 
			GPIOB->ODR |= (1U << 9); // EN=1
			LCD_delay_us(40);
			GPIOB->ODR &= ~(1U << 9);//EN=0
			LCD_delay_us(40);
			// Send lower nibble
			LCD_send_Bit(Char<<4, mode); 
			GPIOB->ODR |= (1U << 9); // EN=1
			LCD_delay_us(40);
			GPIOB->ODR &= ~(1U << 9);//EN=0
			LCD_delay_us(40);
	}
}

void LCD_sendString(char *str, LCD_Mode mode) {
    while (*str) {
        LCD_sendChar(*str++, mode);
    }
}
void LCD_init(LCD_Mode mode){
	LCD_GPIO_Init();    
	LCD_delay_ms(50);
	if(mode == b8)	
		LCD_sendCMD(0x38, mode); // turn on 8 bit - 2 lines
	else{
		LCD_sendCMD(0x02, mode); 
	  LCD_sendCMD(0x28, mode); // turn on 4 bit - 2 lines
	}
		LCD_sendCMD(0x0C, mode); // Display on, cursor off
    LCD_sendCMD(0x01, mode); // Clear display
		LCD_sendCMD(0x06, mode); // Entry mode set: Increment cursor
	}
void LCD_SetCursor(int row,int num, LCD_Mode mode){
	if(row <0 || row >1 || num <0 || num>15) return;
	if (row ==0){	
		LCD_sendCMD(0x80, mode); 
		LCD_sendCMD(0x80+num, mode); 
	}
	else {
		LCD_sendCMD(0xC0, mode); 
		LCD_sendCMD(0xC0+num, mode); }
}	

void lcd_clear(LCD_Mode mode)
{
   LCD_sendCMD(0x01,mode);   // Goi ham truyen thong tin (lenh) sang C.LCD.
        // Lenh: Xoa hien thi tren man hinh C.LCD (0x01).
}

void intToStr(int num, char* str) {
    int i = 0;
    char temp[10]; // Temporary buffer to store digits
    int isNegative = 0;

    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return;
    }

    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    while (num != 0) {
        temp[i++] = (num % 10) + '0';
        num /= 10;
    }

    if (isNegative) temp[i++] = '-'; 

    temp[i] = '\0';

    // Reverse the string
    int j = 0;
    for (int k = i - 1; k >= 0; k--) {
        str[j++] = temp[k];
    }
    str[j] = '\0';
}