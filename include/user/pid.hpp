#ifndef _PID_HPP
#define _PID_HPP


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


void pidInitialize (PID pid, float kP, float kI, float kD, float integral_threshold);
float pidCalculate (PID pid, float setpoint, double encoderVariable);


extern float power;


#endif
