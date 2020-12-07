#ifndef __IMU_H
#define __IMU_H

// for MPU6000

#include "stm32f4xx_hal.h"

// MPU6000 Register Map
#define WHO_AM_I 							0x75
#define Accel_Xout_H				0x3B
#define SIGNAL_PATH_RESET		0x68
#define USER_CTRL						0x6A
#define PWR_MGMT_1					0x6B
#define PWR_MGMT_2					0x6C

#define IMU_CS_PORT					GPIOA
#define IMU_CS_PIN					GPIO_PIN_8
#define IMU_CS_HIGH					HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET)
#define IMU_CS_LOW					HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET)


void IMU_Init(void);
void IMU_Reg_Write(uint8_t reg, uint8_t value);
void IMU_Reg_Read(uint8_t reg, uint8_t *buf, uint8_t size);
void imuRead(int16_t *acc, int16_t *temperature, int16_t *gyro);

#endif
