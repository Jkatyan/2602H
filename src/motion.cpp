#include "main.h"


int wait_until(bool (*condition)(void*), int timeOut){
  int start_time = millis();
  void* buffer = lv_mem_alloc( 4 * sizeof(int) );

  while( millis() - start_time < timeOut ){
    if( (*condition)(buffer) ){
      return 0;
    }
  }
  return -1;
}


bool is_motor_at_target(Motor motor, int error){
  return abs( (int)( motor.get_target_position() - motor.get_position() )  ) < error;
}


bool is_all_chassis_motors_at_target(){
  return//:
    is_motor_at_target(LD_F, CHASSIS_MAX_ERROR) &&
    is_motor_at_target(RD_F, CHASSIS_MAX_ERROR) &&
    is_motor_at_target(LD_R, CHASSIS_MAX_ERROR) &&
    is_motor_at_target(RD_R, CHASSIS_MAX_ERROR);
}


bool chassis_holding_at_target(void* buffer){
  int* longest_hold = (int*) buffer;

  if( *longest_hold < 0 ){
    longest_hold = 0;
  }

  if( !is_all_chassis_motors_at_target() ){
    *longest_hold = millis();
  }
}


int autonomous_motion(struct Autonomous_Section* section){

  autonomous_section_motion_type_e type = section -> movement_type;
  int len = section -> length;
  int spd = section -> speed;
  int timeOut = section -> timeOut;

  if( type == move ){
    LD_F.move_relative(len, spd);
    RD_F.move_relative(len, spd);
    LD_R.move_relative(len, spd);
    RD_R.move_relative(len, spd);
    wait_until( &chassis_holding_at_target, timeOut );
  }

  if( type == turn ){
    LD_F.move_relative(len, spd);
    RD_F.move_relative((-1)*len, spd);
    LD_R.move_relative(len, spd);
    RD_R.move_relative((-1)*len, spd);
    wait_until( &chassis_holding_at_target, timeOut );   
  }
}
