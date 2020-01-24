#include "main.h"


bool is_motor_at_target(Motor motor, int error){
  return abs( (int)( motor.get_target_position() - motor.get_position() )  ) < error;
}


bool is_all_chassis_motors_at_target(){
  return//:
    is_motor_at_target(LD, CHASSIS_MAX_ERROR);
}


int chassis_autonomous_motion(autonomous_section_motion_type_e motion_type, int length, int speed, int timeOut){
  int motion_start_time = millis();

  while( millis() - motion_start_time < timeOut ){
    if( is_all_chassis_motors_at_target() ){
      return 0;
    }
    delay(20);
  }
  return -1;
}



void motion_initialize(){
  ;/*pass*/
}
