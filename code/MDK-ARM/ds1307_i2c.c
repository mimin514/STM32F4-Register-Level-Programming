//#include "ds1307_i2c.h"

//// Hàm kh?i t?o I2C
//void I2C1_Init(void) {
//    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   // Enable clock I2C1
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable clock GPIOB

//    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);  // Clear bits
//    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // Alternate Function
//    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;      // Open-Drain
//    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7; // High Speed
//    GPIOB->AFR[0] |= (4 << 24) | (4 << 28);  // AF4 cho I2C1 trên PB6 và PB7

//    I2C1->CR1 = 0;  // Reset Control Register 1
//    I2C1->CR2 = 16; // Set clock to 16MHz
//    I2C1->CCR = 80; // Clock control (100kHz)
//    I2C1->TRISE = 17; // Rise time

//    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C
//}

//// G?i d? li?u I2C
//void I2C1_Write(uint8_t device_address, uint8_t register_address, uint8_t data) {
//    I2C1->CR1 |= I2C_CR1_START;
//    while (!(I2C1->SR1 & I2C_SR1_SB));
//    
//    I2C1->DR = device_address & ~0x01;
//    while (!(I2C1->SR1 & I2C_SR1_ADDR));
//    (void) I2C1->SR2;

//    I2C1->DR = register_address;
//    while (!(I2C1->SR1 & I2C_SR1_TXE));

//    I2C1->DR = data;
//    while (!(I2C1->SR1 & I2C_SR1_TXE));

//    I2C1->CR1 |= I2C_CR1_STOP;
//}

//// Nh?n d? li?u I2C
//uint8_t I2C1_Read(uint8_t device_address, uint8_t register_address) {
//    uint8_t data;

//    I2C1->CR1 |= I2C_CR1_START;
//    while (!(I2C1->SR1 & I2C_SR1_SB));

//    I2C1->DR = device_address & ~0x01;
//    while (!(I2C1->SR1 & I2C_SR1_ADDR));
//    (void) I2C1->SR2;

//    I2C1->DR = register_address;
//    while (!(I2C1->SR1 & I2C_SR1_TXE));

//    I2C1->CR1 |= I2C_CR1_START;
//    while (!(I2C1->SR1 & I2C_SR1_SB));

//    I2C1->DR = device_address | 0x01;
//    while (!(I2C1->SR1 & I2C_SR1_ADDR));
//    (void) I2C1->SR2;

//    while (!(I2C1->SR1 & I2C_SR1_RXNE));
//    data = I2C1->DR;

//    I2C1->CR1 |= I2C_CR1_STOP;

//    return data;
//}

//// Ghi d? li?u vào DS1307
//void DS1307_Write(uint8_t register_address, uint8_t data) {
//    I2C1_Write(0xD0, register_address, data);
//}

//// Ð?c d? li?u t? DS1307
//uint8_t DS1307_Read(uint8_t register_address) {
//    return I2C1_Read(0xD0, register_address);
//}


//#define DS1307_ADDRESS 0x68

//void DS1307_SetTime(uint8_t hour, uint8_t minute, uint8_t second) {
//    I2C_Start();
//    I2C_Address(DS1307_ADDRESS, 0); // 0 d? ghi
//    I2C_Write(0x00); // Ð?t con tr? vào thanh ghi 00h (giây)
//    I2C_Write(second); 
//    I2C_Write(minute); 
//    I2C_Write(hour); 
//    I2C_Stop();
//}

//void DS1307_GetTime(char* timeStr) {
//    uint8_t sec, min, hour;
//    
//    I2C_Start();
//    I2C_Address(DS1307_ADDRESS, 0); // 0 d? ghi
//    I2C_Write(0x00); // Ð?t con tr? vào thanh ghi 00h (giây)
//    I2C_Stop();

//    I2C_Start();
//    I2C_Address(DS1307_ADDRESS, 1); // 1 d? d?c
//    sec = I2C_Read_Ack();
//    min = I2C_Read_Ack();
//    hour = I2C_Read_NAck();
//    I2C_Stop();

//    // Chuy?n d?i t? BCD sang s? th?p phân
//    sec = (sec >> 4) * 10 + (sec & 0x0F);
//    min = (min >> 4) * 10 + (min & 0x0F);
//    hour = (hour >> 4) * 10 + (hour & 0x0F);

//    // Ð?nh d?ng chu?i th?i gian "HH:MM:SS"
//    sprintf(timeStr, "%02d:%02d:%02d", hour, min, sec);
//}
