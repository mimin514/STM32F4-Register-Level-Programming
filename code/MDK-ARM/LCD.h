#ifndef LCD_H_
#define LCD_H_

#include "stm32f4xx.h"

typedef enum {b4,b8} LCD_Mode;

void LCD_sendCMD(char cmd, LCD_Mode mode);		//gui 1 lenh den LCD
void LCD_sendChar(char Char, LCD_Mode mode); 	// gui 1 ki tu
void LCD_sendString(char *str, LCD_Mode mode);//gui chuoi
void LCD_init(LCD_Mode mode);  								// che do 4bit / 8bit
void LCD_SetCursor(int row,int num, LCD_Mode mode); // set vi tri
void lcd_clear(LCD_Mode mode);
void intToStr(int num, char* str);
#endif /* LCD_H_ */
