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
void setDrive(int left, int right);
void rotate(int degrees, int voltage);
float slewRateCalculate(float desiredRate);
void driveTarget(int target, int time, float speed);
void A_gyroDriveTarget(float angle, int target, int time, float speed);
void A_gyroTurn(int target, int accuracy, int time, float speed);

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
