#include "main.h"

/*
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


bool chassis_at_target(void* buffer){
  return CHASSIS.is_at_target();
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
    if( CHASSIS.is_at_target() ){
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

  movement_e_t type = section -> movement_type;
  int len = section -> length;
  int spd = section -> speed;
  int timeOut = section -> timeOut;

  if( type == move ){
    CHASSIS.move_relative(len, spd);
    if( wait_until( &chassis_holding_at_target, timeOut ) == -1 ){
      ;//pass, do display later
    }
  }

  else if( type == turn ){
    CHASSIS.turn_relative(len, spd);
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

  else if( type == wait ){
    Task::delay(timeOut);
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


void opcontrol_motion(){
  ;
}


void pid_set_all(){
  CHASSIS.set_vel_pid( VEL_P_CHASSIS, VEL_I_CHASSIS, VEL_D_CHASSIS );
  CHASSIS.set_pos_pid( POS_P_CHASSIS, POS_I_CHASSIS, POS_D_CHASSIS );
}


void motion_initialize(){
  CHASSIS.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  LIFT.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  TILTER.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  INTAKE_L.set_brake_mode( E_MOTOR_BRAKE_HOLD );
  INTAKE_R.set_brake_mode( E_MOTOR_BRAKE_HOLD );

  pid_set_all();
}
*/

void apply_powerCurve(double* x, double c = 2){
  if(x >= 0){
    *x = pow(*x, c);
  }
  else{
    *x = (-1) * pow( abs(*x),c );
  }
}

namespace Hybot{
  namespace API{
    void chassis_drive_arcade(double x, double r){
      Components::CHASSIS->arcade(x, r);
    }
    void chassis_drive_arcade_powerCurve(double x, double r){

      Components::CHASSIS->arcade(x, r);
    }
  }
}
