# scat_microcontroller

This repo contains code that is being run on the STM32 on the wheelchair. The microcontroller is in charge of velocity PID control of each motor with feedforward.
It also receives velocity commands from ROS, and sending IMU & velocity & joystick data to ROS.

The backbone of this project's code (most functions that begin with MX_ and Init functions) were generated by STMCubeMX, then additional user code is written in sections
that were designated for user code, between the comment indicators 'USER CODE BEGIN' & 'USER CODE END' at multiple locations. Additional user code was also added in 
separate header and source files.

Feel free to use STMCubeMX's code generation to create new initialization code for new ports, it will not delete user added source and header files, nor will it delete any code
that was added in the user code sections of main.c. Take note that if you disable a certain port, (eg SPI 1,3,6) because they have additional user code in the init functions for 
SPI 1,3,6, they will get removed.

## Sensor Block Diagram
![block_diagram](motor_sensor.png)

## Motor Control Block Diagram
![block_diagram](motor_control.png)
