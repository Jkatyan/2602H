#ifndef _AUTONOMOUS_HPP
#define _AUTONOMOUS_HPP

#include "motion.hpp"


struct Autonomous_Section{
  movement_e_t movement_type;
  int length;
  int speed;
  int timeOut;
};


extern struct Autonomous_Section* AUTONOMOUS_SEQUENCE;


void autonomous_initialize();

void load_autonomous();


#endif
