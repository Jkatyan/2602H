#include "main.h"


struct Autonomous_Section* AUTONOMOUS_SEQUENCE;


void autonomous_proceed(struct Autonomous_Section* sequence){
  for(int i = 0; i < 64; i++){
    if( autonomous_motion(sequence + i) ){
      return;
    }
  }
}


void autonomous_initialize(){
  AUTONOMOUS_SEQUENCE = (struct Autonomous_Section*)( lv_mem_alloc( 64 * sizeof(Autonomous_Section) ) );
  load_autonomous();
}


void autonomous(){
  autonomous_proceed( AUTONOMOUS_SEQUENCE );
}
