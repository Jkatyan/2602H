#ifndef AUTONS_HPP
#define AUTONS_HPP

#include "main.h"

const bool DISABLE_AUTONOMOUS = false;

//S_armsMotion_proceed{
const int TILTER_MAX_VALUE = 0;
const int TILTER_MIN_VALUE = -2950;
const int TILTER_SPEED = 127;
const int LIFT_MAX_VALUE = 1300;
const int LIFT_MIN_VALUE = 0;
const int LIFT_SPEED = 127;
const int INTAKEA_SPEED = 127;
const int INTAKEB_SPEED = 127;
//}

//backup_autonomous_API
const int AUTOMOVE_ALLOWABLE_ERROR = 36;
const int AUTOMOVE_SUCCESS_HOLDING_TIME = 500;
const int LCD_DISPLAY_FRAMERATE = 30;
//backup_autonomous_API

extern PID drivePID;
extern PID turnPID;

extern float lastSlewTime;
extern float maxAccel;
extern float lastSlewRate;

void goTo(float targetX, float targetY);
/*
Goes to a specific coordinate using ODOMETRY

*targetX - X coordinate
*targetY - Y coordinate
*/
void setDrive(int left, int right);
/*
Helper function for Chassis

*left - Set left side power
*right - Set right side power
*/
void rotate(int degrees, int voltage);
/*
Gyro based turning

*degrees - Amount of degrees you want to turn
*voltage - Controls chassis voltage (Ex. 127 for max speed)
*/
float slewRateCalculate(float desiredRate);
/*
A basic 1D motion profiler / Acceleration Control

*desiredRate - Max acceleration allowed. (Ex. 0.16)
*/
void driveTarget(int target, int time, float speed);
/*
Straight Line Motion

*Target - Desired Target Value
*Time - Allowed time for the function to run before it ends (ms)
*Speed - Speed multiplier for the drive (Ex. 0.5 50% speed, 1 100% speed)
*/
void A_gyroDriveTarget(float angle, int target, int time, float speed);
/*
Straight Line Motion with Gyro Correction.

*Angle - Uses Gyro to correct angle while driving / RELATIVE TO STARTING ALIGNMENT.
*Target - Desired Target Value
*Time - Allowed time for the function to run before it ends (ms)
*Speed - Speed multiplier for the drive (Ex. 0.5 50% speed, 1 100% speed)
PID and Slew applied by default.
*/
void A_gyroTurn(int target, int accuracy, int time, float speed);
/*
Gyro based turning with PID and Slew

*Target - Angle of Turn
*Accuracy - How close to the target the robot is
*Time - Amount of time allowed for the turn. (In MS)
*Speed - Speed coefficient. Ex. 1 is max speed, 0.5 is half speed.
*/
bool S_motorSuccess( pros::Motor motor );
int S_chassis_wait_till_success(int timeOut, int mode);
int S_chassis_move(int angle, float speed, int timeOut);
int S_chassis_turn(int angle, float speed, int timeOut);
void S_zero_all_motors();
void S_reset_all_motors();
void S_drive_chassis_tank();
void S_drive_chassis_arcade();
void S_moveMotor_withLimit(pros::Motor motor, int velocity, int max_value, int min_value,
pros::controller_digital_e_t button1, pros::controller_digital_e_t button2, int limitSource);
void S_armsMotion_proceed();
void S_armsMotion_Amode_proceed();

#endif
