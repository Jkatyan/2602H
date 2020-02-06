#ifndef _PID_HPP
#define _PID_HPP

void pidInitialize (PID pid, float kP, float kI, float kD, float integral_threshold);
float pidCalculate (PID pid, float setpoint, float encoderVariable);

typedef struct{
float kP;
float kI;
float kD;
  float error;
  float previousError;
  float integral;
  float integral_threshold;
  float derivative;
  float target;
} PID;


extern float power;


#endif