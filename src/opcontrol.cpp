#include "main.h"


struct Key_Binding* KEY_MAP;


void opcontrol_initialize(){
	KEY_MAP = (struct Key_Binding*)( lv_mem_alloc( 64 * sizeof(Autonomous_Section) ) );
	load_keyBindings();
}


void opcontrol() {
	while(1){
		/*pass*/;
	}
}
