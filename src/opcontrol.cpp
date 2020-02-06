#include "main.h"


void opcontrol() {
	while(1){
		LD_F.move( controller0.get_analog(ANALOG_LEFT_Y) );
		LD_R.move( controller0.get_analog(ANALOG_LEFT_Y) );
		RD_F.move( controller0.get_analog(ANALOG_RIGHT_Y) );
		RD_R.move( controller0.get_analog(ANALOG_RIGHT_Y) );
		controller0.print(0, 0, "%d    %d    ", (int)(LD_F.get_torque()*1000), (int)(RD_F.get_torque()*1000));
		controller0.print(1, 0, "%d    %d    ", (int)(LD_R.get_torque()*1000), (int)(RD_R.get_torque()*1000));
		delay(200);
	}
}
