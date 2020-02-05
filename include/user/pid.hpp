#ifndef PID_HPP
#define PID_HPP

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

float power; //Output Value

#endif
