
#include <imu.h>

extern SPI_HandleTypeDef hspi3;

void IMU_Init(void)
{
	HAL_Delay(100);
	uint8_t rxWhoAmI;
	IMU_Reg_Read(WHO_AM_I, &rxWhoAmI, 1);

	if(rxWhoAmI != 0x68)
	{
		__NOP();			// Cannot detect MPU6000, do something here if needed
	}

	HAL_Delay(100);
	// Initialize MPU6000
	IMU_Reg_Write(PWR_MGMT_1, 0X80);
	HAL_Delay(100);
	IMU_Reg_Write(SIGNAL_PATH_RESET, 0X07);
	HAL_Delay(100);
	IMU_Reg_Write(PWR_MGMT_1, 0X01);
	IMU_Reg_Write(PWR_MGMT_2, 0X00);
}

void IMU_Reg_Write(uint8_t reg, uint8_t value)
{
	IMU_CS_LOW;
	HAL_SPI_Transmit(&hspi3, &reg, 1, 1);
	HAL_SPI_Transmit(&hspi3, &value, 1, 1);
	IMU_CS_HIGH;
}

void IMU_Reg_Read(uint8_t reg, uint8_t *buf, uint8_t size)
{
	reg |= 0x80;
//	uint8_t all_reg[size];
//	all_reg[0] = reg;
//	if(size > 1)
//	{
//		for(int i = 1; i < size; ++i)
//			all_reg[i] = ++reg;
//	}

	IMU_CS_LOW;
	HAL_Delay(100);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi3, &reg, 1, 1);
//	HAL_Delay(100);
	ret = HAL_SPI_Receive(&hspi3, buf, size, 1);
	IMU_CS_HIGH;
}

void imuRead(int16_t *acc, int16_t *temperature, int16_t *gyro)
{
	static uint8_t rxBuff[14];
	// Accel(3*2 bytes) + Temperature(1*2 bytes) + Gyro(3*2 bytes)

	IMU_Reg_Read(Accel_Xout_H, rxBuff, 14);

	acc[0] = (uint16_t)(rxBuff[0]) << 8 | rxBuff[1];
	acc[1] = (uint16_t)(rxBuff[2]) << 8 | rxBuff[3];
	acc[2] = (uint16_t)(rxBuff[4]) << 8 | rxBuff[5];

	*temperature = (uint16_t)(rxBuff[6]) << 8 | rxBuff[7];
	//double temprature_actual= double 36.53+((double)temperature)/340;

	gyro[0] = (uint16_t)(rxBuff[8]) << 8 | rxBuff[9];
	gyro[1] = (uint16_t)(rxBuff[10]) << 8 | rxBuff[11];
	gyro[2] = (uint16_t)(rxBuff[12]) << 8 | rxBuff[13];
}

