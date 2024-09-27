#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "GPIO.h"
#include "LCD.h"
#include "delay.h"
#include "LED.h"
#include "LED_matrix.h"
#include <cmath>
#include "adc.h"
#include "PWM.h"
#include "encoder.h"
#include "UART.h"
//#include "interrupt.h"
#include "SPI.h"
///////		GPIO		//////////////////////////////////
//int main(void) {  

//		SystemInit();
//	delay_init();
//    LED_Configuration();
//    while (1) {
//        	LedBlink();
////        delay_ms(500);
//    }
//}
/////////////////////////////////////////////////



/////		LCD 4 bit - 8 bit		//////////////////////////////////

//int main(void)
//{
//	SystemInit();
//	//delay_init();
//	// **********************4 bit **************
//	 LCD_init(b4);
//    // Initialize LCD in 4-bit mode
//		LCD_SetCursor(0,3,b4);
//		LCD_sendString("Ng Ngoc Ha My", b4);
//    LCD_SetCursor(1,0,b4);
//    LCD_sendString("4-bit nha", b4);
//	
//	// **********************8 bit **************
////		LCD_init(b8);
////    // Initialize LCD in 8-bit mode
////		LCD_SetCursor(0,3,b8);
////		LCD_sendString("2212104", b8);
////    LCD_SetCursor(1,0,b8);
////    LCD_sendString("8-bit nha", b8);
//	
//	// *********************************************
//    while (1) {
//        // Infinite loop
//			
//    }
//} 
/////////////////////////////////////////////////



/////		LED	7 DOAN	//////////////////////////////////

//int main(void) {
//    SystemInit(); 
//    LED_7_GPIO_Init();
//	
//    while (1) {
////			
//			for(int i=0;i<=9999;i++){
//				//for(int j=0;j<20;j++)
//			display__7(i);
//				//LCD_delay_ms(1000);  
//			}
////			for(int i=0;i<10;i++){
////				for(int j=0;j<4;j++){
////					selectDigit(j);
////					selectNum(i);
////				}
////			}
//		}
//}
/////////////////////////////////////////////////



/////////		LED	MATRIX	//////////////////////////////////
//int main(void) {
//    SystemInit();
////delay_init();	
//		LED_matrix_GPIO_Init();
//	uint8_t off[8] = {0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C};//mat cuoi
//unsigned char  chu[][8]={{0x00,0x66,0xFF,0xFF,0x7E,0x3C,0x18,0x00}};//k
//uint8_t maled[8] = 
////{0x18, 0x3C, 0x66, 0x66, 0x7E, 0x7E, 0x66, 0x66};//A
////{0x00,0x66,0xFF,0xFF,0x7E,0x3C,0x18,0x00};//heart
//{0x63,0x66,0x6C,0x78,0x78,0x6c,0x66,0x63}; //K
////{0x00,0xC3,0xE7,0xFF,0xDB,0xC3,0xC3,0xC3};//M
////{0x00,0xC3,0xE3,0xF3,0xDB,0xCF,0xC7,0xC3};//N
////{0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C};//mat cuoi
////{0x0B,0x7E,0xD8,0x18,0x18,0x24,0x42,0xC3};//he huoc

//uint8_t maledd[8] = {0x00,0x66,0xFF,0xFF,0x7E,0x3C,0x18,0x00};//heart


////		 chaychu(chu[0], 8);
//while (1)
//    {
//			 chaychu(chu[0], 8);
//				//display_matrix_run( off);
//       //display_matrix(maledd);
//		//display_matrix(maled);
//	}
//}
/////////////////////////////////////////////////



///////		ADC  	//////////////////////////////////

//int main()
//{
//SystemInit();
//	ADC_Init();
//	LCD_init(b4);


//	while(1)
//	{
//				run_adc(1);// 1 for nhiet do do am, 2 for var, 3 for x y
//		//run_real();
//	}
//}
/////////////////////////////////////////////////



/////////		PWM  	//////////////////////////////////

//int main()
//{
//SystemInit();
//timer4_init();
//timer1_Init();
//while (1) {
//run_pwm(1);// 1 for led, else for motor 
//}
//}


///////////////////////////////////////////////////


/////////		ENCODER  	//////////////////////////////////

//int main()
//{
//		SystemInit();
//    encoder_init();
//	run_encoder();
//    while (1) {
//    }
//}

///////////////////////////////////////////////////


/////////		UART  	//////////////////////////////////
int main(void) {
    // Initialize GPIO and UART
    GPIO_Init();
    UART_Init(9600);  // Initialize UART with 9600 baud rate

    // Send a string over UART
    char* message = "Hello, UART communication!";
    Send_string_uart(message);

    // Infinite loop
    while (1) {
        // Optionally, echo received data back
        uint8_t received = receive_data();
        transmit_data(received);
    }

    return 0;
}


///////////////////////////////////////////////////

/////////		INTERRUPT  	//////////////////////////////////


//int main(void) {
//    GPIO_Configuration();
//    TIM2_Configuration();
//	EXTI0_Configuration();
//    //UART2_Configuration();
////delay_init();
//    while (1) {

//    }
//		}
///////////////////////////////////////////////////


/////////		SPI - MAX7219  	//////////////////////////////////

//int main(void) {
//    SPI1_Init();
//    GPIO_Init_spi();
//    USART2_Init();

//    MAX7219_Init();

//    while (1) {
//        DisplayNumber(98765432);
//    }
//}

///////////////////////////////////////////////////










////#include "stm32f4xx.h"
//#include "stdio.h"
////#define DS3231_ADDR     0x68   // DS3231 I2C address
////#define I2C_TIMEOUT     50000  // Timeout for I2C operations
////#define DS1307_ADDRESS 0xD0
////// Function prototypes
////void I2C_Init(void);
////void I2C_Start(void);
////void I2C_Stop(void);
////void I2C_Write(uint8_t data);
////uint8_t I2C_Read_Ack(void);
////uint8_t I2C_Read_NAck(void);
////void I2C_Address(uint8_t address, uint8_t direction);

////void DS3231_Write(uint8_t reg, uint8_t data);
////uint8_t DS3231_Read(uint8_t reg);

////// Low-level I2C functions
//void I2C_Init(void) {
//    // Enable clocks for GPIOB and I2C1
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

//    // Configure PB8 (SCL) and PB9 (SDA) as alternate function I2C
//    GPIOB->MODER |= (0x2 << 16) | (0x2 << 18);    // Alternate function for PB8, PB9
//    GPIOB->AFR[1] |= (0x4 << 0) | (0x4 << 4);     // Set AF4 (I2C1) for PB8 and PB9

//    // Configure I2C
////    I2C1->CR1 = 0;
////    I2C1->CR2 = 0x10;  // APB1 frequency is 16 MHz
////    I2C1->CCR = 80;    // 100 kHz standard mode
////    I2C1->TRISE = 17;  // Maximum rise time
////    I2C1->CR1 |= I2C_CR1_PE;  // Enable I2C1
//	  I2C1->CR1 &= ~I2C_CR1_PE; // Vô hi?u hóa I2C
//    I2C1->CR2 = 0x0020;       // T?n s? I2C (20 MHz / 32 = 625 kHz)
//    I2C1->CCR = 0x0080;       // C?u hình t?c d? I2C
//    I2C1->TRISE = 0x0021;     // Th?i gian rise
//    I2C1->CR1 |= I2C_CR1_PE;  // B?t I2C
//}
////// Hàm g?i byte qua I2C
////void I2C_SendByte(uint8_t data) {
////    I2C1->DR = data;               // Ðua d? li?u vào d? li?u register
////    while (!(I2C1->SR1 & I2C_SR1_TXE)); // Ch? cho d? li?u du?c g?i
////}

////// Hàm nh?n byte t? I2C
////uint8_t I2C_ReceiveByte(void) {
////    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Ch? d? li?u d?n
////    return I2C1->DR; // Ð?c d? li?u
////}

////// Hàm vi?t d? li?u vào DS1307
////void DS1307_Write(uint8_t reg, uint8_t data) {
////    I2C1->CR1 |= I2C_CR1_START;   // G?i tín hi?u START
////    while (!(I2C1->SR1 & I2C_SR1_SB)); // Ch? START du?c g?i

////    I2C_SendByte(DS1307_ADDRESS); // G?i d?a ch? DS1307
////    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Ch? d?a ch? du?c g?i

////    (void)I2C1->SR2; // Ð?c SR2 d? xóa c?

////    I2C_SendByte(reg); // G?i d?a ch? register
////    I2C_SendByte(data); // G?i d? li?u

////    I2C1->CR1 |= I2C_CR1_STOP; // G?i tín hi?u STOP
////}

////// Hàm d?c d? li?u t? DS1307
////uint8_t DS1307_Read(uint8_t reg) {
////    I2C1->CR1 |= I2C_CR1_START;   // G?i tín hi?u START
////    while (!(I2C1->SR1 & I2C_SR1_SB)); // Ch? START du?c g?i

////    I2C_SendByte(DS1307_ADDRESS); // G?i d?a ch? DS1307
////    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Ch? d?a ch? du?c g?i

////    (void)I2C1->SR2; // Ð?c SR2 d? xóa c?

////    I2C_SendByte(reg); // G?i d?a ch? register

////    I2C1->CR1 |= I2C_CR1_START | I2C_CR1_ACK; // G?i tín hi?u START l?i d? chuy?n sang ch? d? d?c
////    while (!(I2C1->SR1 & I2C_SR1_SB)); // Ch? START du?c g?i

////    I2C1->DR = DS1307_ADDRESS | 0x01; // G?i d?a ch? DS1307 v?i bit d?c
////    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Ch? d?a ch? du?c g?i

////    (void)I2C1->SR2; // Ð?c SR2 d? xóa c?

////    uint8_t data = I2C_ReceiveByte(); // Nh?n d? li?u
////    I2C1->CR1 |= I2C_CR1_STOP; // G?i tín hi?u STOP

////    return data;
////}

////int main(void) {
////    I2C_Init(); // Kh?i t?o I2C

////    // Ví d?: Ghi giá tr? 0x12 vào register 0x00 c?a DS1307
////    DS1307_Write(0x00, 0x12);

////    // Ví d?: Ð?c giá tr? t? register 0x00 c?a DS1307
////    uint8_t value = DS1307_Read(0x00);

////    while (1) {
////        // Chuong trình chính
////    }
////}







////#include "stm32f4xx.h"

////// C?u hình I2C1
////void I2C1_Init(void) {
////    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Kích ho?t d?ng h? cho I2C1
////    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Kích ho?t d?ng h? cho GPIOB

////    // C?u hình PB6 (SCL) và PB7 (SDA) cho I2C1
////    GPIOB->MODER |= (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1); // Alternate function mode
////    GPIOB->AFR[0] |= (0x04 << (6 * 4)) | (0x04 << (7 * 4));    // AF4 cho I2C1

////    I2C1->CR1 = I2C_CR1_PE; // Enable I2C
////    I2C1->CR2 = 0x0020; // Set I2C speed to 100kHz
////    I2C1->CCR = 0x0030; // Configure I2C timing
////    I2C1->TRISE = 0x0021; // Set rise time
////}

////// G?i byte qua I2C
////void I2C_SendByte(uint8_t data) {
////    I2C1->DR = data;
////    while (!(I2C1->SR1 & I2C_SR1_TXE)); // Ch? TXE c?
////}

////// Nh?n byte qua I2C
////uint8_t I2C_ReceiveByte(void) {
////    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Ch? RXNE c?
////    return I2C1->DR;
////}

////// Vi?t d? li?u vào DS1307
////void DS1307_Write(uint8_t reg, uint8_t data) {
////    I2C1->CR1 |= I2C_CR1_START;
////    while (!(I2C1->SR1 & I2C_SR1_SB));

////    I2C_SendByte(0xD0); // DS1307 d?a ch? write
////    while (!(I2C1->SR1 & I2C_SR1_ADDR));
////    (void)I2C1->SR2;

////    I2C_SendByte(reg);
////    I2C_SendByte(data);

////    I2C1->CR1 |= I2C_CR1_STOP;
////}

////// Ð?c d? li?u t? DS1307
////uint8_t DS1307_Read(uint8_t reg) {
////    I2C1->CR1 |= I2C_CR1_START;
////    while (!(I2C1->SR1 & I2C_SR1_SB));

////    I2C_SendByte(0xD0); // DS1307 d?a ch? write
////    while (!(I2C1->SR1 & I2C_SR1_ADDR));
////    (void)I2C1->SR2;

////    I2C_SendByte(reg);

////    I2C1->CR1 |= I2C_CR1_START;
////    while (!(I2C1->SR1 & I2C_SR1_SB));

////    I2C1->DR = 0xD1; // DS1307 d?a ch? read
////    while (!(I2C1->SR1 & I2C_SR1_ADDR));
////    (void)I2C1->SR2;

////    uint8_t data = I2C_ReceiveByte();
////    I2C1->CR1 |= I2C_CR1_STOP;

////    return data;
////}

////void DisplayTimeOnLCD(void) {
////    uint8_t seconds = DS1307_Read(0x00);
////    uint8_t minutes = DS1307_Read(0x01);
////    uint8_t hours = DS1307_Read(0x02);

////    char buffer[16];
////    sprintf(buffer, "Time: %02x:%02x:%02x", hours, minutes, seconds);

////    LCD_sendCMD(0x80,8); // Move cursor to beginning of the first line
////    for (int i = 0; buffer[i] != '\0'; i++) {
////        LCD_sendChar(buffer[i],8);
////    }
////}

////int main(void) {
////    I2C1_Init();
////    LCD_init(b8);

////    while (1) {
////        DisplayTimeOnLCD();
////        for (volatile int i = 0; i < 1000000; i++); // Delay
////    }
////}












////#include "stm32f4xx.h"

////// C?u hình I2C1
////void I2C1_Init(void) {
////    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Kích ho?t d?ng h? cho I2C1
////    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Kích ho?t d?ng h? cho GPIOB

////    // C?u hình PB6 (SCL) và PB7 (SDA) cho I2C1
////    GPIOB->MODER |= (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1); // Ch? d? ch?c nang thay th?
////    GPIOB->AFR[0] |= (0x04 << (6 * 4)) | (0x04 << (7 * 4));    // AF4 cho I2C1

////    I2C1->CR1 = 0; // Ð?m b?o I2C không du?c b?t
////    I2C1->CR2 = 0x0020; // Set I2C speed to 100kHz
////    I2C1->CCR = 0x0030; // Configure I2C timing
////    I2C1->TRISE = 0x0021; // Set rise time
////    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C
////}

////// G?i byte qua I2C
////void I2C_SendByte(uint8_t data) {
////    I2C1->DR = data;
////    while (!(I2C1->SR1 & I2C_SR1_TXE)); // Ch? TXE c?
////}

////// Nh?n byte qua I2C
////uint8_t I2C_ReceiveByte(void) {
////    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Ch? RXNE c?
////    return I2C1->DR;
////}

////// Vi?t d? li?u vào DS1307
////void DS1307_Write(uint8_t reg, uint8_t data) {
////    I2C1->CR1 |= I2C_CR1_START;
////    while (!(I2C1->SR1 & I2C_SR1_SB)); // Ch? SB c?

////    I2C_SendByte(0xD0); // DS1307 d?a ch? write
////    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Ch? ADDR c?
////    (void)I2C1->SR2;

////    I2C_SendByte(reg);
////    I2C_SendByte(data);

////    I2C1->CR1 |= I2C_CR1_STOP;
////}

////// Ð?c d? li?u t? DS1307
////uint8_t DS1307_Read(uint8_t reg) {
////    I2C1->CR1 |= I2C_CR1_START;
////    while (!(I2C1->SR1 & I2C_SR1_SB)); // Ch? SB c?

////    I2C_SendByte(0xD0); // DS1307 d?a ch? write
////    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Ch? ADDR c?
////    (void)I2C1->SR2;

////    I2C_SendByte(reg);

////    I2C1->CR1 |= I2C_CR1_START;
////    while (!(I2C1->SR1 & I2C_SR1_SB)); // Ch? SB c?

////    I2C1->DR = 0xD1; // DS1307 d?a ch? read
////    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Ch? ADDR c?
////    (void)I2C1->SR2;

////    uint8_t data = I2C_ReceiveByte();
////    I2C1->CR1 |= I2C_CR1_STOP;

////    return data;
////}

////int main(void) {
////    I2C1_Init();

////    while (1) {
////        // Ví d?: Ð?c giá tr? gi?, phút, giây t? DS1307
////        uint8_t seconds = DS1307_Read(0x00);
////        uint8_t minutes = DS1307_Read(0x01);
////        uint8_t hours = DS1307_Read(0x02);

////        // Ð? ki?m tra, có th? thêm các do?n mã d? xu?t giá tr? ra debug, UART, ho?c dùng debugger c?a Proteus
////        // Ví d?: Gán giá tr? vào các bi?n ho?c g?i qua UART (n?u có)

////        for (volatile int i = 0; i < 1000000; i++); // Delay d? d? dàng theo dõi trong Proteus
////    }
////}
