#ifndef AUTONS_HPP
#define AUTONS_HPP

#include <main.h>

bool S_motorSuccess( pros::Motor motor );
int S_chassis_wait_till_success(int timeOut, int mode);
int S_chassis_move(int angle, float speed, int timeOut);
int S_chassis_turn(int angle, float speed, int timeOut);
void S_zero_all_motors();
void S_drive_chassis_tank();
void S_drive_chassis_arcade();
void S_moveMotor_withLimit(pros::Motor motor, int velocity, int max_value, int min_value,
pros::controller_digital_e_t button1, pros::controller_digital_e_t button2, int limitSource);

#endif
