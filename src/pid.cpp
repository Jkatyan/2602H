#include "main.h"


float power;


void pidInitialize (PID pid, float kP, float kI, float kD, float integral_threshold) {
pid.kP = kP;
pid.kI = kI;
pid.kD = kD;
  pid.integral = 0;
  pid.integral_threshold = integral_threshold;
}


float pidCalculate (PID pid, float setpoint, double encoderVariable) {
  pid.error = setpoint - encoderVariable;
  pid.integral += pid.error;
  if (pid.error == 0 || pid.integral >= setpoint) {
    pid.integral = 0;
  }
  if (fabs (pid.error) >= pid.integral_threshold) {
    pid.integral = 0;
  }
  pid.derivative = pid.error - pid.previousError;
  pid.previousError = pid.error;
  power = pid.error * pid.kP + pid.integral * pid.kI + pid.derivative * pid.kD;
  power = fabs(power) > 127 ? 127 * power/fabs(power) : power;
  return power;
  pros::delay(10);
}