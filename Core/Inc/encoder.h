#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"
#include <main.h>

#define ENCODER1_CS_PORT		GPIOA
#define ENCODER1_CS_PIN			GPIO_PIN_4
#define ENCODER2_CS_PORT		GPIOG
#define ENCODER2_CS_PIN			GPIO_PIN_8

#define ENCODER_MAX				16384

#define ENCODER1_CS_HIGH		HAL_GPIO_WritePin(ENCODER1_CS_PORT, ENCODER1_CS_PIN, GPIO_PIN_SET)
#define ENCODER1_CS_LOW			HAL_GPIO_WritePin(ENCODER1_CS_PORT, ENCODER1_CS_PIN, GPIO_PIN_RESET)
#define ENCODER2_CS_HIGH		HAL_GPIO_WritePin(ENCODER2_CS_PORT, ENCODER2_CS_PIN, GPIO_PIN_SET)
#define ENCODER2_CS_LOW			HAL_GPIO_WritePin(ENCODER2_CS_PORT, ENCODER2_CS_PIN, GPIO_PIN_RESET)

void encoderRead(uint16_t *encoder_vals);
void calcVelFromEncoder(uint16_t *encoder_vals, double *velocities);

#endif
