#ifndef _AUTONOMOUS_HPP
#define _AUTONOMOUS_HPP


enum autonomous_section_motion_type_e{
  move, turn, lift, tilter, intake, lift_sync, tilter_sunc, end
};


struct Autonomous_Section{
  autonomous_section_motion_type_e movement_type;
  int length;
  int speed;
  int timeOut;
};


extern struct Autonomous_Section* AUTONOMOUS_SEQUENCE;


void autonomous_initialize();


#endif
