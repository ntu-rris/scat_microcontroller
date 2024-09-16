/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <imu.h>
#include <encoder.h>
#include <pid.h>
#include <math.h>
#include <stdio.h>
//#include <bno055.h>
#include <dwt_delay.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROS_UART huart2
#define BRAKE_TIM htim4
#define BRAKE_CHANNEL CCR4
#define MOTOR_TIM htim4
#define LEFT_MOTOR_CHANNEL CCR3
#define RIGHT_MOTOR_CHANNEL CCR1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi6;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//Variables to store raw data from various sensors and uart
uint32_t joystick_raw;
int16_t acc[3], gyro[3];
uint16_t encoder[2];
uint8_t data_from_ros_raw[SIZE_DATA_FROM_ROS] = { 0 };

uint16_t e_stop = 1;

//Variables to store processed data
uint16_t joystick[2];
double joystick_filter = 0.025;
float velocity[2];
int16_t data_from_ros[SIZE_DATA_FROM_ROS / 2];

int16_t motor_command[2] = { 0 };
uint16_t data_to_ros[SIZE_DATA_TO_ROS] = { 0 };
uint8_t braked = 1; //Stores the brake status of left and right motors
double filtered_setpoint[2] = { 0 };
double setpoint_vel[2];
uint32_t brake_timer = 0;
uint32_t prev_uart_time = 0;
double engage_brakes_timeout = 5; //5s
double angular_output = 0;

//Velocity stuff
double target_heading, curr_heading;

//PID struct and their tunings. There's one PID controller for each motor
PID_Struct left_pid, right_pid, left_ramp_pid, right_ramp_pid, left_d_ramp_pid,
		right_d_ramp_pid;
double p = 0.0, i = 100.0, d = 0.0, f = 340, max_i_output = 40;
double pid_freq = 500;

//Feedforward outputs from experimental data, at x speed, what feedforward factor is required
//0.1 = 500, 0.2 = 450, 0.5 = 350, 0.7 = 320, 0.9 = 320
//Feedforward equation to line fit = y=290(x-0.9)^2 + 310

//Added for spi stm32
typedef union {
	float b32;		//original data
	uint8_t b8[4];	//data in byte size
} floatuint8_t;

uint32_t total_receive_num = 0;
uint32_t correct_receive_num = 0;
uint32_t incorrect_receive_num = 0;
uint32_t reset_num = 0;
uint8_t spi_rx_buf[11] = { 0 };
volatile floatuint8_t spi_enc[2] = { 0 };
uint32_t spi_rx_t = 0;
uint32_t diff_t = 0;
float prev_vel[2] = {0};
#define EXPONENTIAL_ALPHA 0.85

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI6_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void setBrakes();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	//Configure systick_callback rate and registration
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / FREQUENCY);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	//Delay required for SPI encoder to startup
	HAL_Delay(100);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_SPI6_Init();
	MX_SPI4_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
//  DWT_Init();
	//Start motor PWM pins
	HAL_TIM_Base_Start(&MOTOR_TIM);
	HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_3);

	//Start brake PWM pins
	HAL_TIM_PWM_Start(&BRAKE_TIM, TIM_CHANNEL_4);

	//Engage brakes
	BRAKE_TIM.Instance->BRAKE_CHANNEL = 1000;

	//Initialize IMU, check that it is connected
	IMU_Init();

	//Initialize BNO055
//  BNO055Init();

	//Start UART output
	HAL_UART_Transmit_DMA(&ROS_UART, (uint8_t*) data_to_ros,
			(uint16_t) SIZE_DATA_TO_ROS * 2);
	HAL_UART_Receive_DMA(&ROS_UART, data_from_ros_raw, SIZE_DATA_FROM_ROS);

	//********* WHEEL PID *********//
	double base_left_ramp_rate = 100;
	double base_right_ramp_rate = 100;
	double base_left_d_ramp_rate = 150;
	double base_right_d_ramp_rate = 150;

	//Setup right wheel PID
	PID_Init(&right_pid);
	PID_setPIDF(&right_pid, p, i, d, f);
	PID_setMaxIOutput(&right_pid, max_i_output);
	PID_setOutputLimits(&right_pid, -500, 500);
	PID_setFrequency(&right_pid, pid_freq);
	PID_setOutputRampRate(&right_pid, base_right_ramp_rate);
	PID_setOutputDescentRate(&right_pid, -base_right_d_ramp_rate);

	//Setup left wheel PID
	PID_Init(&left_pid);
	PID_setPIDF(&left_pid, p, i, d, f);
	PID_setMaxIOutput(&left_pid, max_i_output);
	PID_setOutputLimits(&left_pid, -500, 500);
	PID_setFrequency(&left_pid, pid_freq);
	PID_setOutputRampRate(&left_pid, base_left_ramp_rate);
	PID_setOutputDescentRate(&left_pid, -base_left_d_ramp_rate);

	//********* WHEEL ACCEL RAMP PID *********//
	double right_ramp_p = 100;
	double left_ramp_p = 100;
	double max_ramp_rate_inc = 300;
	//Setup right wheel ramp PID
	PID_Init(&right_ramp_pid);
	PID_setPIDF(&right_ramp_pid, right_ramp_p, 0, 0, 0);
	PID_setOutputLimits(&right_ramp_pid, 0, 400);
	PID_setOutputRampRate(&right_ramp_pid, max_ramp_rate_inc);
	PID_setOutputDescentRate(&right_ramp_pid, -max_ramp_rate_inc);
	PID_setFrequency(&right_ramp_pid, pid_freq);

	//Setup left wheel ramp PID
	PID_Init(&left_ramp_pid);
	PID_setPIDF(&left_ramp_pid, left_ramp_p, 0, 0, 0);
	PID_setOutputLimits(&left_ramp_pid, 0, 400);
	PID_setOutputRampRate(&left_ramp_pid, max_ramp_rate_inc);
	PID_setOutputDescentRate(&left_ramp_pid, -max_ramp_rate_inc);
	PID_setFrequency(&left_ramp_pid, pid_freq);

	//********* WHEEL DECEL RAMP PID *********//
	double right_d_ramp_p = 100;
	double left_d_ramp_p = 150;
	double right_max_d_ramp_rate_inc = 200;
	double left_max_d_ramp_rate_inc = 200;
	double max_d_increase = 550;
	//Setup right wheel d ramp PID
	PID_Init(&right_d_ramp_pid);
	PID_setPIDF(&right_d_ramp_pid, right_d_ramp_p, 0, 0, 0);
	PID_setOutputLimits(&right_d_ramp_pid, 0, max_d_increase);
	PID_setOutputRampRate(&right_d_ramp_pid, right_max_d_ramp_rate_inc);
	PID_setOutputDescentRate(&right_d_ramp_pid, -right_max_d_ramp_rate_inc);
	PID_setFrequency(&right_d_ramp_pid, pid_freq);

	//Setup left wheel d ramp PID
	PID_Init(&left_d_ramp_pid);
	PID_setPIDF(&left_d_ramp_pid, left_d_ramp_p, 0, 0, 0);
	PID_setOutputLimits(&left_d_ramp_pid, 0, max_d_increase);
	PID_setOutputRampRate(&left_d_ramp_pid, left_max_d_ramp_rate_inc);
	PID_setOutputDescentRate(&left_d_ramp_pid, -left_max_d_ramp_rate_inc);
	PID_setFrequency(&left_d_ramp_pid, pid_freq);

	HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, sizeof(spi_rx_buf));
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t prev_time = HAL_GetTick();
	while (1) {
		//Loop should execute once every 1 tick
		if (HAL_GetTick() - prev_time >= 1) {
			imuRead(acc, gyro, 0.2);
			e_stop = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

			if (incorrect_receive_num > 100) {
				//Restart transmission
				HAL_SPI_Abort_IT(&hspi1);
				HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, sizeof(spi_rx_buf));
				incorrect_receive_num = 0;
				reset_num++;
			}
			else if(HAL_GetTick() - spi_rx_t  > 100){
				HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, sizeof(spi_rx_buf));
			}

			//use speed from data_from_ros array, pass on to motors, ensure the data is valid by checking end bit
			if ((uint16_t) data_from_ros[SIZE_DATA_FROM_ROS / 2 - 1]
					== 0xFFFB) {
				//Data received from ros is integer format, multiplied by 1000
				setpoint_vel[LEFT_INDEX] = (double) data_from_ros[0] / 1000.0;
				setpoint_vel[RIGHT_INDEX] = (double) data_from_ros[1] / 1000.0;

				//Heading is synonymous to radius of curvature for given velocity pair
				double target_angular = (setpoint_vel[RIGHT_INDEX]
						- setpoint_vel[LEFT_INDEX]);
				double curr_angular = (velocity[RIGHT_INDEX]
						- velocity[LEFT_INDEX]);
				double target_linear = (setpoint_vel[RIGHT_INDEX]
						+ setpoint_vel[LEFT_INDEX]) / 2.0;
				double curr_linear = (velocity[RIGHT_INDEX]
						+ velocity[LEFT_INDEX]) / 2.0;
				target_heading = atan2(target_linear, target_angular);
				curr_heading = atan2(curr_linear, curr_angular);

				//This case might happen when curr_heading is M_PI and target_heading is -M_PI
				//In this case, both values should be equal signs
				if (target_heading == M_PI || curr_heading == M_PI) {
					curr_heading = fabs(curr_heading);
					target_heading = fabs(target_heading);
				}

				//When angular_output negative, right wheel is slower
				//When angular_output positive, left wheel is slower
				double new_angular_output = (target_heading - curr_heading)
						/ M_PI;
				int sign = new_angular_output / fabs(new_angular_output);

				//Sigmoid curve to make new_angular_output more sensitive in mid range (~0.5)
				//~0.5 is max value that occurs when going from pure rotation to pure forward
				new_angular_output = 1
						/ (1 + exp(-15 * (fabs(new_angular_output) - 0.35)))
						* sign;

				//Small velocities cause large changes to heading due to noise
				//Set difference to 0 if below threshold, no correction
				if (fabs(velocity[LEFT_INDEX]) < 0.1
						&& fabs(velocity[RIGHT_INDEX]) < 0.1)
					new_angular_output = 0;

				double angular_output_filt = 0.0;
				angular_output = angular_output_filt * (angular_output)
						+ (1 - angular_output_filt) * new_angular_output;
				//Amount of penalty to setpoint depends on how far away from the target heading
				//Scale may increase over 100%, but does not matter as the heading approaches target heading
				//scale will approach 100%
				if (setpoint_vel[LEFT_INDEX] != 0.0
						|| setpoint_vel[RIGHT_INDEX] != 0.0) {
					setpoint_vel[LEFT_INDEX] *= (1 + angular_output);
					setpoint_vel[RIGHT_INDEX] *= (1 - angular_output);
				}

				/**
				 * Motor FEEDFORWARD params
				 */
				double f_left = 290
						* pow(fabs(setpoint_vel[LEFT_INDEX]) - 0.9, 2) + 310;
				double f_right = 290
						* pow(fabs(setpoint_vel[RIGHT_INDEX]) - 0.9, 2) + 310;

				//Upper bound on feedforward equation
				if (setpoint_vel[LEFT_INDEX] > 1.0)
					f_left = 310;
				if (setpoint_vel[RIGHT_INDEX] > 1.0)
					f_right = 310;

				PID_setF(&left_pid, f_left);
				PID_setF(&right_pid, f_right);

				//If e stop engaged, override setpoints to 0
				if (e_stop == 1) {
					setpoint_vel[LEFT_INDEX] = 0;
					setpoint_vel[RIGHT_INDEX] = 0;
				}

				//If data is old, set setpoint to 0
				else if ((HAL_GetTick() - prev_uart_time) > FREQUENCY * 0.2) {
					setpoint_vel[LEFT_INDEX] = 0;
					setpoint_vel[RIGHT_INDEX] = 0;
					HAL_UART_Receive_DMA(&ROS_UART, data_from_ros_raw,
					SIZE_DATA_FROM_ROS);
				}

				//Unbrake motors if there is command, brake otherwise
				setBrakes();

				//Ensure there is a commanded velocity, otherwise reset PID
				if (fabs(setpoint_vel[LEFT_INDEX]) == 0
						&& fabs(velocity[LEFT_INDEX]) < 0.05) {
					motor_command[LEFT_INDEX] = 0;
					PID_reset(&left_pid);
				}

				else if (!braked) {
					//ACCELERATE
					{
						double new_left_ramp = base_left_ramp_rate
								+ PID_getOutput(&left_ramp_pid,
										fabs(velocity[LEFT_INDEX]),
										fabs(setpoint_vel[LEFT_INDEX]));
						PID_setOutputRampRate(&left_pid, new_left_ramp);
					}

					//DECELERATE
					{
						double new_left_ramp = base_left_d_ramp_rate
								+ PID_getOutput(&left_d_ramp_pid,
										fabs(setpoint_vel[LEFT_INDEX]),
										fabs(velocity[LEFT_INDEX]));
						PID_setOutputDescentRate(&left_pid, -new_left_ramp);
					}

					motor_command[LEFT_INDEX] = PID_getOutput(&left_pid,
							velocity[LEFT_INDEX], setpoint_vel[LEFT_INDEX]);
				}

				//Ensure there is a commanded velocity, otherwise reset PID
				if (fabs(setpoint_vel[RIGHT_INDEX]) == 0
						&& fabs(velocity[RIGHT_INDEX]) < 0.05) {
					motor_command[RIGHT_INDEX] = 0;
					PID_reset(&right_pid);
				}

				else if (!braked) {

					//ACCELERATE
					{
						double new_right_ramp = base_right_ramp_rate
								+ PID_getOutput(&right_ramp_pid,
										fabs(velocity[RIGHT_INDEX]),
										fabs(setpoint_vel[RIGHT_INDEX]));
						PID_setOutputRampRate(&right_pid, new_right_ramp);
					}

					//DECELERATE
					{
						double new_right_ramp = base_right_d_ramp_rate
								+ PID_getOutput(&right_d_ramp_pid,
										fabs(setpoint_vel[RIGHT_INDEX]),
										fabs(velocity[RIGHT_INDEX]));
						PID_setOutputDescentRate(&right_pid, -new_right_ramp);
					}

					motor_command[RIGHT_INDEX] = PID_getOutput(&right_pid,
							velocity[RIGHT_INDEX], setpoint_vel[RIGHT_INDEX]);
				}

				//Send PID commands to motor
				MOTOR_TIM.Instance->RIGHT_MOTOR_CHANNEL =
						motor_command[LEFT_INDEX] + 1500;
				MOTOR_TIM.Instance->LEFT_MOTOR_CHANNEL =
						motor_command[RIGHT_INDEX] + 1500;

			}

			prev_time = HAL_GetTick();
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */
	HAL_ADC_Start_DMA(&hadc1, &joystick_raw, 2);
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void) {

	/* USER CODE BEGIN SPI4_Init 0 */

	/* USER CODE END SPI4_Init 0 */

	/* USER CODE BEGIN SPI4_Init 1 */

	/* USER CODE END SPI4_Init 1 */
	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI4_Init 2 */

	/* USER CODE END SPI4_Init 2 */

}

/**
 * @brief SPI6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI6_Init(void) {

	/* USER CODE BEGIN SPI6_Init 0 */

	/* USER CODE END SPI6_Init 0 */

	/* USER CODE BEGIN SPI6_Init 1 */

	/* USER CODE END SPI6_Init 1 */
	/* SPI6 parameter configuration*/
	hspi6.Instance = SPI6;
	hspi6.Init.Mode = SPI_MODE_MASTER;
	hspi6.Init.Direction = SPI_DIRECTION_2LINES;
	hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi6.Init.NSS = SPI_NSS_SOFT;
	hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi6.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI6_Init 2 */

	/* USER CODE END SPI6_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 90 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 20000 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
//	huart2.Init.BaudRate = 576000;
	huart2.Init.BaudRate = 230400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PG8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//If adc callback by joystick adc
	if (hadc == &hadc1) {
		//Both channels of ADC (channel 0 and channel 1) are each 16 bit
		//Both are stored in joystick_raw, a 32bit unsigned int
		//Separate both readings in the below code into 2 variables
		joystick[1] = joystick[1] * (1 - joystick_filter)
				+ (joystick_raw & 0x0000FFFF) * joystick_filter;
		joystick[0] = joystick[0] * (1 - joystick_filter)
				+ (joystick_raw >> 16) * joystick_filter;
	}
}

// UART data reception callback function
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static int huart_synchronized = 0;
	static uint8_t synchronize_data;
	if (huart == &ROS_UART) {
		//Serial connection to ROS might have been made midway through a full packet
		//Therefore, read the packets 1 by 1 until the end byte is reached.
		//Once reached, then uart from ros to stm32 is synchronized, read all bytes as normal after that
		if (huart_synchronized < 2) {
			//LSB First, check byte is 0xFFFB
			if (huart_synchronized == 0) {
				if (synchronize_data == 0xFB)
					huart_synchronized++;
			}

			else {
				if (synchronize_data == 0xFF)
					huart_synchronized++;
			}

			//Check if uart is still not synchronized. If not, then continue receiving single bytes
			if (huart_synchronized < 2) {
				HAL_UART_Receive_DMA(&ROS_UART, &synchronize_data, 1);
				return;
			}

			//Else start receiving all the bytes from ROS like normal
			else {
				HAL_UART_Receive_DMA(&ROS_UART, data_from_ros_raw,
				SIZE_DATA_FROM_ROS);
				return;
			}
		}

		//Receive buffer is already synchronized
		else {
			//Mark previous UART time
			prev_uart_time = HAL_GetTick();

			//Combine check bit to ensure uart is still synchronized, otherwise reset synchronized flag
			data_from_ros[SIZE_DATA_FROM_ROS / 2 - 1] =
					(int16_t) data_from_ros_raw[SIZE_DATA_FROM_ROS - 1] << 8
							| data_from_ros_raw[SIZE_DATA_FROM_ROS - 2];
			if ((uint16_t) data_from_ros[SIZE_DATA_FROM_ROS / 2 - 1]
					== 0xFFFB) {
				//Combine 8bit data to 16bit, for use in main while loop
				for (int i = 0; i < SIZE_DATA_FROM_ROS; i += 2)
					data_from_ros[i / 2] = (int16_t) data_from_ros_raw[i + 1]
							<< 8 | (int16_t) data_from_ros_raw[i];

				//Receive next data from ROS
				HAL_UART_Receive_DMA(&ROS_UART, data_from_ros_raw,
				SIZE_DATA_FROM_ROS);
				return;
			}

			//Data is no longer synchronized, reset
			else {
				huart_synchronized = 0;
				HAL_UART_Receive_DMA(&ROS_UART, &synchronize_data, 1);
				return;
			}

		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	//Callback triggeredby TX complete to ROS
	if (huart == &ROS_UART) {
		//Joystick values are raw ADC values, integer format
		data_to_ros[0] = joystick[1];
		data_to_ros[1] = joystick[0];

		//Convert velocity to integer for transferring. ROS node will divide the result by 1000
		data_to_ros[2] = (int16_t) (velocity[LEFT_INDEX] * 1000);
		data_to_ros[3] = (int16_t) (velocity[RIGHT_INDEX] * 1000);

		//Acceleration and gyro is in raw format from IMU, int16_t
		data_to_ros[4] = acc[0];
		data_to_ros[5] = acc[1];
		data_to_ros[6] = acc[2];

		data_to_ros[7] = gyro[0];
		data_to_ros[8] = gyro[1];
		data_to_ros[9] = gyro[2];

		data_to_ros[10] = e_stop;

		data_to_ros[11] = (uint16_t) 0xabcd;

		//Send data to ros again
		//Force STM32 to treat data_to_ros as a uint8_t pointer array as that is what's required.
		//Data will get sent along as normal, then ROS end can combine 2 bytes of data to get original data
		HAL_UART_Transmit_DMA(&ROS_UART, (uint8_t*) data_to_ros,
				(uint16_t) SIZE_DATA_TO_ROS * 2);
	}
}

void setBrakes() {
	if ((setpoint_vel[LEFT_INDEX] != 0 || setpoint_vel[RIGHT_INDEX] != 0)) {
		BRAKE_TIM.Instance->BRAKE_CHANNEL = 2000;
		braked = 0;
		brake_timer = 0;
	}

	else if (setpoint_vel[LEFT_INDEX] == 0 && setpoint_vel[RIGHT_INDEX] == 0) {
		if (fabs(velocity[LEFT_INDEX]) < 0.05
				&& fabs(velocity[RIGHT_INDEX]) < 0.05) {
			//Start timer before braking
			if (brake_timer == 0)
				brake_timer = HAL_GetTick();

			else if (HAL_GetTick()
					- brake_timer> engage_brakes_timeout * FREQUENCY) {
				BRAKE_TIM.Instance->BRAKE_CHANNEL = 1000;
				braked = 1;
				brake_timer = 0;
			}
		}
	}
}
static double filt_vel[2] = {
		0
	};
	static double unfilt_vel[2] = {
			0
		};
	static double prev_velo[2] = {
		0
	};

uint32_t unsync = 0;
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	/* NOTE : This function should not be modified, when the callback is needed,
	 the HAL_SPI_RxCpltCallback should be implemented in the user file
	 */


	if (hspi == &hspi1) {
		total_receive_num += 1;
		uint16_t checksum = 0;
		for (int i = 0; i < 8; i++)
			checksum += spi_rx_buf[i];

		//If SPI is not in sync
		if (HAL_GetTick() - spi_rx_t> 1){
			unsync+=1;
			spi_rx_t = HAL_GetTick();
			HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, sizeof(spi_rx_buf));
			return;
		}


		if (spi_rx_buf[0] == 0xAE && (spi_rx_buf[9] == (uint8_t) (checksum & 0xff))
				&& (spi_rx_buf[10] == (uint8_t) ((checksum >> 8) & 0xff))) {
			    floatuint8_t spi_enc[2];
			    spi_enc[LEFT_INDEX].b8[0] = spi_rx_buf[1];
			    spi_enc[LEFT_INDEX].b8[1] = spi_rx_buf[2];
			    spi_enc[LEFT_INDEX].b8[2] = spi_rx_buf[3];
			    spi_enc[LEFT_INDEX].b8[3] = spi_rx_buf[4];
			    spi_enc[RIGHT_INDEX].b8[0] = spi_rx_buf[5];
			    spi_enc[RIGHT_INDEX].b8[1] = spi_rx_buf[6];
			    spi_enc[RIGHT_INDEX].b8[2] = spi_rx_buf[7];
			    spi_enc[RIGHT_INDEX].b8[3] = spi_rx_buf[8];
			//Correct the sign
			spi_enc[LEFT_INDEX].b32 *= -1;
			diff_t = HAL_GetTick() - spi_rx_t;
			spi_rx_t = HAL_GetTick();
			correct_receive_num += 1;

			//Exponential filter for each velocity
			unfilt_vel[LEFT_INDEX] = spi_enc[LEFT_INDEX].b32 * WHEEL_DIA * 0.5;
			unfilt_vel[RIGHT_INDEX] = spi_enc[RIGHT_INDEX].b32 * WHEEL_DIA * 0.5;

		    // Sometimes data gets lost and spikes are seen in the velocity readouts.
		    // This is solved by limiting the max difference between subsequent velocity readouts.
		    // If acceleration is passed, just update velocity within acceleration limits
//		    double dt = (double) diff_t / FREQUENCY;

//		    if (dt != 0) {
//			double right_acc = (unfilt_vel[RIGHT_INDEX] - prev_velo[RIGHT_INDEX]) / dt;
//			double left_acc = (unfilt_vel[LEFT_INDEX] - prev_velo[LEFT_INDEX]) / dt;
//
//			if (fabs(right_acc) > WHEEL_ACC_LIMIT) {
//			    unfilt_vel[RIGHT_INDEX] = prev_velo[RIGHT_INDEX]
//				    + WHEEL_ACC_LIMIT * dt * (right_acc / fabs(right_acc));
//			}
//
//			if (fabs(left_acc) > WHEEL_ACC_LIMIT) {
//			    unfilt_vel[LEFT_INDEX] = prev_velo[LEFT_INDEX] + WHEEL_ACC_LIMIT * dt * (left_acc / fabs(left_acc));
//
//		    }
//		    }
			velocity[LEFT_INDEX] = 0.85 * filt_vel[LEFT_INDEX] + 0.075 * unfilt_vel[LEFT_INDEX]
				+ 0.075 * prev_velo[LEFT_INDEX];
			velocity[RIGHT_INDEX] = 0.85 * filt_vel[RIGHT_INDEX] + 0.075 * unfilt_vel[RIGHT_INDEX]
				+ 0.075 * prev_velo[RIGHT_INDEX];

			velocity[LEFT_INDEX] = (fabs(velocity[LEFT_INDEX]) < 0.01) ? 0 : velocity[LEFT_INDEX];
			velocity[RIGHT_INDEX] = (fabs(velocity[RIGHT_INDEX]) < 0.01) ? 0 : velocity[RIGHT_INDEX];
			filt_vel[LEFT_INDEX] = velocity[LEFT_INDEX];
			filt_vel[RIGHT_INDEX] = velocity[RIGHT_INDEX];
			prev_velo[LEFT_INDEX] = unfilt_vel[LEFT_INDEX];
			prev_velo[RIGHT_INDEX] = unfilt_vel[RIGHT_INDEX];

		} else {
			incorrect_receive_num += 1;
		}

		HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, sizeof(spi_rx_buf));
//	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*) spi_tx_buf, sizeof(spi_tx_buf) );
	}
}

uint16_t error_count = 0;
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_ErrorCallback should be implemented in the user file
   */
  /* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
            and user can use HAL_SPI_GetError() API to check the latest error occurred
   */
  error_count++;
  HAL_SPI_Abort_IT(&hspi1);
  HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, sizeof(spi_rx_buf));

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_ErrorCallback should be implemented in the user file
   */
  /* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
            and user can use HAL_SPI_GetError() API to check the latest error occurred
   */
//  error_count++;
//  HAL_SPI_Abort_IT(&hspi1);
//  				HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, sizeof(spi_rx_buf));

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
