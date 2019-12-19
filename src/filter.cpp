#include "main.h"
#include "autons.hpp"

int* f_buf;

int filterInit(){
  f_buf = (int*) malloc( FILTER_BUFFER_LENGTH * sizeof(int) );
  if( f_buf ) return 0;
  else return -1;
}
