/*
 * bno055.h
 *
 *  Created on: Jun 24, 2021
 *      Author: by
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stm32f4xx_hal.h"
#include <math.h>

void BNO055Read(uint8_t reg, uint8_t* data, uint8_t resp_byte, uint16_t bytes);
void BNO055Write(uint8_t reg, uint8_t* data, uint8_t resp_byte, uint16_t bytes);
void BNO055Init();

#endif /* INC_BNO055_H_ */
