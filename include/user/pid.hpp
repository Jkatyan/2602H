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


extern float power;


#endif
