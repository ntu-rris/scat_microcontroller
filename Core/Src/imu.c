#include <imu.h>
#include <math.h>
//#include <dwt_delay.h>

extern SPI_HandleTypeDef IMU_SPI;

void IMU_Init(void)
{
	uint8_t rxWhoAmI;
	IMU_Reg_Read(WHO_AM_I, &rxWhoAmI, 1);

	if(rxWhoAmI != 0x69)
	{
		__NOP();			// Cannot detect LSM6DS33, do something here if needed
	}

	// Initialize LSM6DS33, sampling rate of 1660Hz
	IMU_Reg_Write(ACC_CONFIG_REG, ACC_1660_HZ | ACC_FS_2G);
	IMU_Reg_Write(GYRO_CONFIG_REG, GYRO_1660_HZ | GYRO_500_DPS);

	//Set accelerometer low pass filter, cutoff frequency = Hz/ODR_X
	IMU_Reg_Write(ACC_FILTER_REG, ACC_LPF_EN | ACC_CUTOFF_ODR_400);
}

void IMU_Reg_Write(uint8_t reg, uint8_t value)
{
	IMU_CS_LOW;
//	DWT_Delay(4);
	HAL_SPI_Transmit(&IMU_SPI, &reg, 1, 1);
//	DWT_Delay(4);
	HAL_SPI_Transmit(&IMU_SPI, &value, 1, 1);
//	DWT_Delay(4);
	IMU_CS_HIGH;
}

int IMU_Reg_Read(uint8_t reg, uint8_t *buf, uint8_t size)
{
	//MSB of read command byte must be 1
	reg = reg | READ;

	for(int i = 0; i < size; ++i)
	{
		IMU_CS_LOW;
//		DWT_Delay(4);
		HAL_SPI_Transmit(&IMU_SPI, &reg, 1, 1);
//		DWT_Delay(4);
		HAL_SPI_Receive(&IMU_SPI, &buf[i], 1, 1);
//		DWT_Delay(4);
		IMU_CS_HIGH;
		++reg;
	}

	return 1;
}

void imuRead(int16_t *acc, int16_t *gyro, double exponentialFilter)
{
	//Check if data is ready
	uint8_t dataReady;
	uint8_t STATUS_REG = 0x1E;
	IMU_Reg_Read(STATUS_REG, &dataReady, 1);
	if(dataReady & 0b010 && dataReady & 0b001){

		// Accel(3*2 bytes) + Gyro(3*2 bytes)
		int bytes = 12;
		uint8_t rxBuff[] = {0,0,0,0,0,0,0,0,0,0,0,0};

		//All data registers are continuous, first register is GYRO_X_LOW
		uint8_t GYRO_X_LOW = 0x22;
		if(!IMU_Reg_Read(GYRO_X_LOW, rxBuff, bytes))
			return;

		//Data is in order xyz
		gyro[0] = gyro[0] * (1.0 - exponentialFilter) + (rxBuff[0] | (int16_t)(rxBuff[1] << 8)) * exponentialFilter;
		gyro[1] = gyro[1] * (1.0 - exponentialFilter) + (rxBuff[2] | (int16_t)(rxBuff[3] << 8)) * exponentialFilter;
		gyro[2] = gyro[2] * (1.0 - exponentialFilter) + (rxBuff[4] | (int16_t)(rxBuff[5] << 8)) * exponentialFilter;

		acc[0] = acc[0] * (1.0 - exponentialFilter) + ((int16_t) rxBuff[6] | (int16_t)(rxBuff[7] << 8)) * exponentialFilter;
		acc[1] = acc[1] * (1.0 - exponentialFilter) + ((int16_t) rxBuff[8] | (int16_t)(rxBuff[9] << 8)) * exponentialFilter;
		acc[2] = acc[2] * (1.0 - exponentialFilter) + ((int16_t) rxBuff[10] | (int16_t)(rxBuff[11] << 8)) * exponentialFilter;

		//Likely that IMU is stuck/hanged (z acceleration acc[2] should be around 16k), attempt to reinitialize IMU
		if(fabs(acc[2]) > 32000)
			IMU_Init();
	}
}

