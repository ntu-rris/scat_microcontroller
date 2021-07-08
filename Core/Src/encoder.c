#include <encoder.h>
#include <math.h>
//#include <dwt_delay.h>

#define MOVING_AVERAGE_SIZE 20
#define EXPONENTIAL_ALPHA 0.85

extern SPI_HandleTypeDef hspi6;			// for Encoders
extern SPI_HandleTypeDef hspi1;			// for Encoders

uint16_t encoder_vals_prev[2] = {-1, -1};
double velocities_prev[2] = {0, 0};
uint32_t prev_time = 0;

// Read Position
// Hex command sequence: 0x00 0x00
// Reset Encoder
// Hex command sequence: 0x00 0x60
// Set Zero Point
// Hex command sequence: 0x00 0x70

void encoderRead(uint16_t *encoder_vals)
{
	uint8_t SPItransmit[2] = {0x00, 0x00};
	uint8_t receive_buff[2];
	uint16_t temp;

	// Encoder 2
	ENCODER2_CS_LOW;

	//Delay required as encoder is only readable after 25us from chip select low
//	DWT_Delay(4);
	HAL_SPI_TransmitReceive(&hspi6, &SPItransmit[0], &receive_buff[0], 1, 1);

	//Delay required between bytes
//	DWT_Delay(4);
	HAL_SPI_TransmitReceive(&hspi6, &SPItransmit[1], &receive_buff[1], 1, 1);

	//There is also a required delay before releasing the CS line
//	DWT_Delay(4);
	ENCODER2_CS_HIGH;
	temp = (uint16_t)receive_buff[0]<< 8 | receive_buff[1];

	//Remove checksum bits (2 MSB bits)
	encoder_vals[RIGHT_INDEX] = temp & 0x3FFF;
	
	// Encoder 1
	ENCODER1_CS_LOW;
//	DWT_Delay(4);
	HAL_SPI_TransmitReceive(&hspi1, &SPItransmit[0], &receive_buff[0], 1, 1);
//	DWT_Delay(4);
	HAL_SPI_TransmitReceive(&hspi1, &SPItransmit[1], &receive_buff[1], 1, 1);
//	DWT_Delay(4);
	ENCODER1_CS_HIGH;
	temp = (uint16_t)receive_buff[0]<< 8 | receive_buff[1];
	encoder_vals[LEFT_INDEX] = temp & 0x3FFF;
}

void calcVelFromEncoder(uint16_t *encoder_vals, double *velocities)
{
	//If previous time is not set or no previous velocities yet, set prev_time and skip this round
	if(prev_time == 0 || (encoder_vals_prev[RIGHT_INDEX] == -1 && encoder_vals_prev[LEFT_INDEX] == -1))
	{
		encoder_vals_prev[RIGHT_INDEX] = encoder_vals[RIGHT_INDEX];
		encoder_vals_prev[LEFT_INDEX] = encoder_vals[LEFT_INDEX];
		prev_time = HAL_GetTick();
		return;
	}

	//Get difference in encoder
	int16_t diff_enc_left = encoder_vals[LEFT_INDEX] - encoder_vals_prev[LEFT_INDEX];
	int16_t diff_enc_right = encoder_vals[RIGHT_INDEX] - encoder_vals_prev[RIGHT_INDEX];

	//Get difference in time
	double dt = (HAL_GetTick() - prev_time) / (double)FREQUENCY;
	if(dt == 0)
		return;

	//Encoders will wrap around, offset the wrap around if it does happen
	//Wrap around is detected by  checking if the difference in encoder value exceeds half the max encoder value
	if (diff_enc_right < -ENCODER_MAX / 2.0)
		diff_enc_right += ENCODER_MAX;
	else if(diff_enc_right > ENCODER_MAX / 2.0)
		diff_enc_right -= ENCODER_MAX;

	if (diff_enc_left < -ENCODER_MAX / 2.0)
		diff_enc_left += ENCODER_MAX;
	else if (diff_enc_left > ENCODER_MAX / 2.0)
		diff_enc_left -= ENCODER_MAX;

	velocities[RIGHT_INDEX] = (double)diff_enc_right / ENCODER_MAX * M_PI * WHEEL_DIA / dt;
	velocities[LEFT_INDEX] = -(double)diff_enc_left / ENCODER_MAX * M_PI * WHEEL_DIA / dt;

//	if(fabs(velocities[RIGHT_INDEX]) < 0.01)
//		velocities[RIGHT_INDEX] = 0.000;
//
//	if(fabs(velocities[LEFT_INDEX]) < 0.01)
//		velocities[LEFT_INDEX] = 0.000;

	// Sometimes data gets lost and spikes are seen in the velocity readouts.
	// This is solved by limiting the max difference between subsequent velocity readouts.
	// If acceleration is passed, just update velocity within acceleration limits
	double right_acc = (velocities[RIGHT_INDEX] - velocities_prev[RIGHT_INDEX]) / dt;
	double left_acc = (velocities[LEFT_INDEX] - velocities_prev[LEFT_INDEX]) / dt;

	if (fabs(right_acc) > WHEEL_ACC_LIMIT)
	{
		velocities[RIGHT_INDEX] = velocities_prev[RIGHT_INDEX] + WHEEL_ACC_LIMIT * dt * (right_acc / fabs(right_acc));
	}

	if (fabs(left_acc) > WHEEL_ACC_LIMIT)
	{
		velocities[LEFT_INDEX] = velocities_prev[LEFT_INDEX] + WHEEL_ACC_LIMIT * dt * (left_acc / fabs(left_acc));
	}

	//Exponential filter for each velocity
	velocities[RIGHT_INDEX] = velocities[RIGHT_INDEX] * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * velocities_prev[RIGHT_INDEX];
	velocities[LEFT_INDEX] = velocities[LEFT_INDEX] * EXPONENTIAL_ALPHA + (1.0 - EXPONENTIAL_ALPHA) * velocities_prev[LEFT_INDEX];

	//Set all previous values to current values
	encoder_vals_prev[RIGHT_INDEX] = encoder_vals[RIGHT_INDEX];
	encoder_vals_prev[LEFT_INDEX] = encoder_vals[LEFT_INDEX];
	velocities_prev[RIGHT_INDEX] = velocities[RIGHT_INDEX];
	velocities_prev[LEFT_INDEX] = velocities[LEFT_INDEX];
	prev_time = HAL_GetTick();
}
