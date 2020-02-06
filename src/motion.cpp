#include "main.h"


void zero_buffer(void* buffer){
  int* p = (int*)buffer;
  for(int i = 0; i < 4; i++){
    *(p+i) = 0;
  }
}


int wait_until(bool (*condition)(void*), int timeOut){
  int start_time = millis();
  void* buffer = lv_mem_alloc( 4 * sizeof(int) );
  zero_buffer(buffer);

  while( millis() - start_time < timeOut ){
    if( (*condition)(buffer) ){
      return 0;
    }
    Task::delay(3);
  }
  return -1;
}


bool motor_at_target(Motor motor, int error){
  return abs( (int)( motor.get_target_position() - motor.get_position() )  ) < error;
}


bool all_chassis_motors_at_target(){
  return//:
    motor_at_target(LD_F, CHASSIS_MAX_ERROR) &&
    motor_at_target(RD_F, CHASSIS_MAX_ERROR) &&
    motor_at_target(LD_R, CHASSIS_MAX_ERROR) &&
    motor_at_target(RD_R, CHASSIS_MAX_ERROR);
}


bool chassis_holding_at_target(void* buffer){
  int* sumTime = (int*)buffer;
  int* lastCalcTime = (int*)buffer + 1;
  int* isInitialized = (int*)buffer + 2;

  int currentTime = millis();

  if( *isInitialized == 0 ){
    *lastCalcTime = currentTime;
    *isInitialized = 1;
  }
  else{
    if( all_chassis_motors_at_target() ){
      if( currentTime - *lastCalcTime > CHASSIS_AUTONMOVE_HOLDTIME ){
        return true;
      }
    }
    else{
      *lastCalcTime = currentTime;
    }
  }
  return false;
}


bool autonomous_motion(struct Autonomous_Section* section){

  autonomous_section_motion_type_e type = section -> movement_type;
  int len = section -> length;
  int spd = section -> speed;
  int timeOut = section -> timeOut;

  if( type == move ){
    LD_F.move_relative(len, spd);
    RD_F.move_relative(len, spd);
    LD_R.move_relative(len, spd);
    RD_R.move_relative(len, spd);
    if( wait_until( &chassis_holding_at_target, timeOut ) == -1 ){
      ;//pass
    }
  }

  else if( type == turn ){
    LD_F.move_relative(len, spd);
    RD_F.move_relative((-1)*len, spd);
    LD_R.move_relative(len, spd);
    RD_R.move_relative((-1)*len, spd);
    if( wait_until( &chassis_holding_at_target, timeOut ) == -1 ){
      ;//pass
    }
  }

  else if( type == lift ){

  }

  else if( type == end ){
    return true;
  }

  return false;

}
