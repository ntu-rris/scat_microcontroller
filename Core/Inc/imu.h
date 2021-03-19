#ifndef __IMU_H
#define __IMU_H
#include "stm32f4xx_hal.h"

// for LSM6DS33
// https://www.st.com/resource/en/datasheet/lsm6ds33.pdf

#define READ				0x80
#define WRITE				0x00

#define WHO_AM_I			0x0F
#define SW_RESET_REG		0x12

#define SW_RESET			0b00000101

//Acceleration config register and available settings
#define ACC_CONFIG_REG 		0x10

//Sampling frequency
#define ACC_12_HZ			0b00010000
#define ACC_104_HZ			0b01000000
#define ACC_208_HZ			0b01010000
#define ACC_833_HZ			0b01110000
#define ACC_1660_HZ			0b10000000
#define ACC_3330_HZ			0b10010000
#define ACC_6660_HZ			0b10100000

//Fullscale g size, signed 16 bit
#define ACC_FS_2G			0b00000000
#define ACC_FS_4G			0b00001000
#define ACC_FS_8G			0b00001100
#define ACC_FS_16G			0b00000100

//Acceleration low/high pass filter register and available settings
#define ACC_FILTER_REG 		0x17

//Enable low pass filter
#define ACC_LPF_EN			0b10000000

//Enable high pass filter
#define ACC_HPF_EN			0b00000100

//LP/HP cutoff frequency
#define ACC_CUTOFF_ODR_50	0b00000000 //this divisor is 9 if HPF is applied
#define ACC_CUTOFF_ODR_100	0b00100000
#define ACC_CUTOFF_ODR_9	0b01000000
#define ACC_CUTOFF_ODR_400	0b01100000


//Gyroscope config register and available settings
#define GYRO_CONFIG_REG		0x11

//Gyro sampling frequency
#define GYRO_104_HZ			0b01000000
#define GYRO_208_HZ			0b01010000
#define GYRO_833_HZ			0b01110000
#define GYRO_1660_HZ		0b10000000

//Gyro degree/s limit, signed 16 bit
#define GYRO_250_DPS		0b00000000
#define GYRO_500_DPS		0b00000100
#define GYRO_1000_DPS		0b00001000
#define GYRO_2000_DPS 		0b00001100


//Defined funtions to set and unset chip select pin
#define IMU_CS_PORT					GPIOA
#define IMU_CS_PIN					GPIO_PIN_8
#define IMU_SPI						hspi4
#define IMU_CS_HIGH					HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET)
#define IMU_CS_LOW					HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET)

void IMU_Init(void);
void IMU_Reg_Write(uint8_t reg, uint8_t value);
int IMU_Reg_Read(uint8_t reg, uint8_t *buf, uint8_t size);
void imuRead(int16_t *acc, int16_t *gyro, double exponentialFilter);

#endif
