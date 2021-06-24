/*
 * bno055.c
 *
 *  Created on: Jun 24, 2021
 *      Author: by
 */
#include <bno055.h>
extern UART_HandleTypeDef huart4;

void BNO055Read(uint8_t reg, uint8_t* data, uint8_t resp_byte, uint16_t bytes)
{
	//Concatenate data
	uint8_t read_command[4];
	read_command[0] = 0xAA;
	read_command[1] = 0x01;
	read_command[2] = reg;
	read_command[3] = (uint8_t) bytes; //Number of bytes to read

	//Transmit READ command
	HAL_UART_Transmit(&huart4, read_command, 4, (uint32_t)100);

	//Receive read data
	uint8_t offset = 2;
	uint8_t response[bytes + offset];
	for(uint8_t i = 0; i < bytes + offset; ++i)
		response[i] = 0x00;

	uint16_t size = bytes + offset;
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart4, response, 2, (uint32_t)10000);

//	for(uint8_t i = 0; i < bytes + offset; ++i)
//		response[i] = 0x00;
//	HAL_UART_Receive(&huart4, response, 1, (uint32_t)100);
//
//	for(uint8_t i = 0; i < bytes + offset; ++i)
//		response[i] = 0x00;
//	HAL_UART_Receive(&huart4, response, 1, (uint32_t)100);

	//Successful read
	if(response[0] == 0xBB && (uint8_t)response[1] == bytes)
	{
		for(uint8_t i = 0; i < bytes; ++i)
			data[i] = response[i + offset];

		resp_byte = 0x00;
	}

	//Fail read
	else if(response[0] == 0xEE)
		resp_byte = 0xAA;
}

void BNO055Write(uint8_t reg, uint8_t* data, uint8_t resp_byte, uint16_t bytes)
{
	//Concatenate data to write
	uint8_t offset = 4;
	uint8_t all_data[bytes + offset];
	all_data[0] = 0xAA; //Start of write command
	all_data[1] = 0x00; //Write command
	all_data[2] = reg; 	//Register to write
	all_data[3] = (uint8_t)bytes;	//How many bytes to write

	for(int i = offset; i < bytes + offset; ++i)
		all_data[i] = data[i - offset];

	//Transmit WRITE data
	HAL_UART_Transmit(&huart4, all_data, bytes + offset, (uint32_t)100);

	//Get WRITE response
	uint8_t response[2];
	HAL_UART_Receive(&huart4, response, 2, (uint32_t)100);

	//If response was valid from BNO055
	if(response[0] == 0xEE)
		resp_byte = response[1];

	else
		resp_byte = 0xAA;
}

void BNO055Init()
{
	//Read address register
	uint8_t addr[1];
	uint8_t resp = 0x00;
	BNO055Read(0x98, addr, resp, 18);

	if(resp == 0xAA)
		return;

	else if(addr[0] == 0xA0)
		return;
}
