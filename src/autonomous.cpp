#include "main.h"


struct Autonomous_Section* AUTONOMOUS_SEQUENCE;


void autonomous_initialize(){
  AUTONOMOUS_SEQUENCE = (struct Autonomous_Section*)( lv_mem_alloc( 64 * sizeof(Autonomous_Section) ) );
}


void autonomous(){
  TEST_MOTOR.move(127);
}
