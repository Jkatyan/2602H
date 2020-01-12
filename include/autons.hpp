#ifndef AUTONS_HPP
#define AUTONS_HPP

#include "main.h"

const bool DISABLE_AUTONOMOUS = false;

//S_armsMotion_proceed{
const int TILTER_MAX_VALUE = 3640;
const int TILTER_MIN_VALUE = 1280;
const int TILTER_SPEED = 127;
const int LIFT_MAX_VALUE = 1000000;
const int LIFT_MIN_VALUE = -1000000;
const int LIFT_SPEED = 127;
const int INTAKEA_SPEED = 127;
const int INTAKEB_SPEED = 127;
//}

//backup_autonomous_API
const int AUTOMOVE_ALLOWABLE_ERROR = 36;
const int AUTOMOVE_SUCCESS_HOLDING_TIME = 200;
const int LCD_DISPLAY_FRAMERATE = 30;
//backup_autonomous_API

extern PID drivePID;
extern PID turnPID;
extern PID tilterPID;
extern PID liftPID;

extern float lastSlewTime;
extern float maxAccel;
extern float maxAccelTray;
extern float lastSlewRate;

const int FILTER_BUFFER_LENGTH = 100;
extern int* f_buf; //Filter-buffer

void A_goTo(float targetX, float targetY);
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
void A_rotate(int degrees, int voltage);
/*
Gyro based turning

*degrees - Amount of degrees you want to turn
*voltage - Controls chassis voltage (Ex. 127 for max speed)
*/
float slewRateCalculate(float desiredRate);
/*
A basic 1D motion profiler / Acceleration Control

*desiredRate - Input value for slewrate to be applied to.
*/
float motorSlew(float desiredRate, float maxAccelMotor);
/*
1D motion profiling for a specific motor.

*desiredRate - Input value for slewrate to be applied to.
*maxAccelMotor - Input max acceleration rate for the motor.
*/
void A_motorTarget(int port, PID pid, int special, int target, int time, float speed, float accel, bool slew);
/*
One motor target movement function w/ PID and slew.

*port - Port of the motor
*pid - Motor's PID **MAKE SURE TO INITIALIZE IN AUTON**
*special - Get the encoder value from a specific source. 0 = IME, 1 = Potentiometer
*target - Specified target
*time - Allowed time in ms
*speed - Speed coefficient
*accel - Maximum acceletation allowed (Ex. 0.16)
*slew - Slew enabled if true
*/
void A_driveTarget(int target, int time, float speed);
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
/*
Check if a motor reached it's target position

parameters:
  -pros::Motor motor - motor to be checked

constants:
  -AUTOMOVE_ALLOWABLE_ERROR - the maximum diffrence between the target position and the real position that will be considered success (for all motors)

return:
  -bool true - if the motor reached it's target position OR
  -bool false - if the motor does not reach it's target position

return after:
  -instantly
*/
int S_chassis_wait_till_success(int timeOut, int mode);
/*
Wait until all chassis motors reached it's target position

parameters:
  -int timeOut - maximum wait time before timeout (in milliseconds)
  -int mode - the movement mode of the chassis currently (0 for move, 1 for turn)

constants:
  -AUTOMOVE_SUCCESS_HOLDING_TIME - the minimum time as holding the chassis in the target position before considering the move being successed
  -LCD_DISPLAY_FRAMERATE - the framerate of the LCD-display

return:
  -int 0 - if the all chassis motors reached it's target position before timeout OR
  -int -1 - if timeOut

return after: (WARNING: this is a synchronous function)
  -all chassis motors reached it's target get_position OR
  -timeout
*/
int S_chassis_move(int angle, float speed, int timeOut);
/*
Move the chassis in a straight line with a specific speed and for a specific distance

parameters:
  -int angle - the total angle required for the motors of the chassis to spin (in degrees)
  -float speed - the required speed for the chassis to move (in proportion, from 0 to 1)
  -int timeout - maximum move attempting time before timeout (in milliseconds)

constants:
  -none

return:
  -int 0 - if the move successed before timeout OR
  -int -1 - if timeout

return after: (WARNING: this is a synchronous function)
  -move successed OR
  -timeout
*/
int S_chassis_turn(int angle, float speed, int timeOut);
/*
Turn the chassis in a specific speed for a specific angle

parameters:
  -int angle - the total angle required for the motors of the chassis to spin (in degrees)
  -float speed - the required speed for the chassis to move (in proportion, from 0 to 1)
  -int timeout - maximum move attempting time before timeout (in milliseconds)

constants:
  -none

return:
  -int 0 - if the turn successed before timeout OR
  -int -1 - if timeout

return after: (WARNING: this is a synchronous function)
  -turn successed OR
  -timeout
*/
void S_zero_all_motors();
/*
Stop all motors from moving (WARNING:using this function inappropriately will cause the motors to malfunction)

parameters:
  -none

constants:
  -none

return:
  -none

return after:
  -instantly
*/
void S_reset_all_motors();
/*
Reset the position of all motors to 0 (WARNING:using this function inappropriately will cause the motors to malfunction)

parameters:
  -none

constants:
  -none

return:
  -none

return after:
  -instantly
*/
void S_reset_chassis_motors();
/*
Reset the position of all chassis motors to 0 (WARNING:using this function inappropriatly will cause the motors to malfunction)

parameters:
  -none

constants:
  -none

return:
  -none

return after:
  -instantly
*/
void S_drive_chassis_tank();
/*
Drive the chassis with a tank-drive (implement: call the function in the main loop, between 20 to 1000 hz)

parameters:
  -none

constants:
  -none

return:
  -none

return after:
  -instantly
*/
void S_drive_chassis_arcade();
/*
Drive the chassis with a arcade-drive (implement: call the function in the main loop, between 20 to 1000 hz)

parameters:
  -none

constants:
  -none

return:
  -none

return after:
  -instantly
*/
void S_moveMotor_withLimit(pros::Motor motor, int velocity, int max_value, int min_value,
pros::controller_digital_e_t button1, pros::controller_digital_e_t button2, int limitSource);
/*
Set the movement to a motor according to the status of two buttons and the position and the limit-position in a specific speed

parameters:
  -pros::Motor motor - motor to be moved
  -int velocity - the speed of the movement (from 0 to 127)
  -int max_value - the maximum allowed position of the motor
  -int min_value - the minimum allowed position of the motor
  -pros::controller_digital_e_t button1 - the button to use to move the motor forward
  -pros::controller_digital_e_t button2 - the button to use to move the motor backward
  -int limitSource - indicate the data source of the motor's position (0 for disable limit, 1 for motor's IMU, 2 for anglemeter, 3 for encoder)
  WARNING: the anglemeter limit source and the encoder limit source is NOT implemented yet (limitSource 2 OR limitSource 3)

constants:
  -none

return:
  -none

return after:
  -instantly
*/
void S_armsMotion_proceed();
/*
Control the arms of the robot in the following way: (implement: call the function in the main loop, between 20 to 1000 hz)
  - button L1 and L2 - tilter
  - button UP and DOWN - lift
  - button R1 and R2 - intake

parameters:
  -none

constants:
  --TILTER_SPEED, TILTER_MAX_VALUE, TILTER_MIN_VALUE, LIFT_SPEED, LIFT_MAX_VALUE, LIFT_MIN_VALUE, INTAKEA_SPEED, INTAKEB_SPEED

return:
  -none

return after:
  -instantly
*/
void S_armsMotion_Amode_proceed();
/*
Control the arms of the robot in the following way: (implement: call the function in the main loop, between 20 to 1000 hz)
  - left joystick - tilter
  - button UP and DOWN - lift
  - button R1 and R2 - intake

parameters:
  -none

constants:
  --LIFT_SPEED, LIFT_MAX_VALUE, LIFT_MIN_VALUE, INTAKEA_SPEED, INTAKEB_SPEED

return:
  -none

return after:
  -instantly
*/











#endif
