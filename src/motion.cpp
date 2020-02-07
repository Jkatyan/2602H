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
    Task::delay( 1000 / AUTONMOVE_WAIT_CHECKFREQ );
  }
  return -1;
}


bool motor_at_target(Motor motor, int error){
  return abs( (int)( motor.get_target_position() - motor.get_position() )  ) < error;
}


bool lift_at_target(void* buffer){
  return motor_at_target(LIFT, LIFT_MAX_ERROR);
}


bool tilter_at_target(void* buffer){
  return motor_at_target(TILTER, TILTER_MAX_ERROR);
}


bool intake_at_target(void* buffer){
  return//:
    motor_at_target(INTAKE_L, INTAKE_MAX_ERROR) &&
    motor_at_target(INTAKE_R, INTAKE_MAX_ERROR);
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
      ;//pass, do display later
    }
  }

  else if( type == turn ){
    LD_F.move_relative(len, spd);
    RD_F.move_relative((-1)*len, spd);
    LD_R.move_relative(len, spd);
    RD_R.move_relative((-1)*len, spd);
    if( wait_until( &chassis_holding_at_target, timeOut ) == -1 ){
      ;//pass, do display later
    }
  }

  else if( type == lift ){
    LIFT.move_absolute(len, spd);
  }

  else if( type == tilter ){
    TILTER.move_absolute(len, spd);
  }

  else if( type == intake_move ){
    INTAKE_L.move(spd);
    INTAKE_R.move(spd);
  }

  else if( type == lift_sync ){
    LIFT.move_relative(len, spd);
    if( wait_until( &lift_at_target, timeOut ) == -1 ){
      ;//pass, do display later
    }
  }

  else if( type == tilter_sync ){
    TILTER.move_relative(len, spd);
    if( wait_until( &tilter_at_target, timeOut ) == -1 ){
      ;//pass, do display later
    }
  }

  else if( type == intake ){
    INTAKE_L.move_relative(len, spd);
    INTAKE_R.move_relative(len, spd);
    if( wait_until( &intake_at_target, timeOut ) == -1 ){
      ;//pass, do display later
    }
  }

  else if( type == end ){
    //do display later
    return true;
  }

  else{
    ;//pass, do display later
  }

  return false;

}

void motion_initialize(){
  LD_F.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  LD_R.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  RD_F.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  RD_R.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  LIFT.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  TILTER.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  INTAKE_L.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  INTAKE_R.set_brake_mode( E_MOTOR_BRAKE_HOLD );
}
