#ifndef I2C_DS1307_H
#define I2C_DS1307_H

#include "stm32f4xx.h"

// �?nh nghia c�c h�m kh?i t?o I2C v� giao ti?p DS1307
void I2C1_Init(void);
void I2C1_Write(uint8_t device_address, uint8_t register_address, uint8_t data);
uint8_t I2C1_Read(uint8_t device_address, uint8_t register_address);
void DS1307_Write(uint8_t register_address, uint8_t data);
uint8_t DS1307_Read(uint8_t register_address);
void DS1307_SetTime(uint8_t hour, uint8_t minute, uint8_t second);
void DS1307_GetTime(uint8_t *hour, uint8_t *minute, uint8_t *second);

#endif // I2C_DS1307_H
