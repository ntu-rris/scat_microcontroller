/*
 * pid.h
 *
 *  Created on: Dec 3, 2020
 *      Author: by
 */

#ifndef __PID_H
#define __PID_H
#include <main.h>

typedef struct PID{
	//Controller parameters
	double P;
	double I;
	double D;
	double F;

	//Limits
	double maxIOutput;
	double maxError;
	double errorSum;
	double prevError;
	double deadTime;
	double frequency;

	double maxOutput;
	double minOutput;

	double setpoint;

	double lastActual;

	//Flags
	int firstRun;
	int reversed;

	//Ramping and descent limits
	double outputRampRate;
	double outputDescentRate;
	double lastOutput;

	double outputFilter;

	double setpointRange;

	uint32_t prev_time;
} PID_Struct;

void PID_Init(PID_Struct* pid);
void checkSigns(PID_Struct* pid);
double clamp(double value, double min, double max);
uint8_t bounded(double value, double min, double max);
void PID_setPID(PID_Struct* pid, double p, double i, double d);
void PID_setPIDF(PID_Struct* pid, double p, double i, double d, double f);
void PID_setMaxIOutput(PID_Struct* pid, double maximum);
void PID_setOutputLimits(PID_Struct* pid, double min, double max);
void PID_setDirection(PID_Struct* pid, int reversed);
void PID_setSetpoint(PID_Struct* pid, double setpoint);
double PID_skipCycle(PID_Struct* pid);
double PID_getOutput(PID_Struct* pid, double actual, double setpoint);
double PID_getOutputFast(PID_Struct* pid);
void PID_reset(PID_Struct* pid);
void PID_setOutputRampRate(PID_Struct* pid, double rate);
void PID_setOutputDescentRate(PID_Struct* pid, double rate);
void PID_setSetpointRange(PID_Struct* pid, double range);
void PID_setOutputFilter(PID_Struct* pid, double strength);
void PID_setFrequency(PID_Struct* pid, double freq);

#endif /* INC_PID_H_ */

























