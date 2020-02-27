#ifndef _MOTION_HPP
#define _MOTION_HPP

#include "api.h"


enum movement_e_t{
  end, move, turn, lift, tilter, intake_move, lift_sync, tilter_sync, intake, wait, tank_drive, arcade_drive
};


/*
bool motor_at_target(pros::Motor, int);

bool autonomous_motion(struct Autonomous_Section*);

void opcontrol_motion();

void motion_initialize();
*/


namespace Hybot{
  namespace API{
    void chassis_drive_arcade(double, double, bool = false, bool = false);
  }
}

#endif
