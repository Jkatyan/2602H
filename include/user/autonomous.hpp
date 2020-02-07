#ifndef _AUTONOMOUS_HPP
#define _AUTONOMOUS_HPP


enum autonomous_section_motion_type_e{
  end, move, turn, lift, tilter, intake_move, lift_sync, tilter_sync, intake, wait
};


struct Autonomous_Section{
  autonomous_section_motion_type_e movement_type;
  int length;
  int speed;
  int timeOut;
};


extern struct Autonomous_Section* AUTONOMOUS_SEQUENCE;


void autonomous_initialize();

void load_autonomous();


#endif
