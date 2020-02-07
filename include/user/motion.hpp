#ifndef _MOTION_HPP
#define _MOTION_HPP

#include "api.h"


bool motor_at_target(pros::Motor, int);

bool autonomous_motion(struct Autonomous_Section*);

void opcontrol_motion();

void motion_initialize();


#endif
