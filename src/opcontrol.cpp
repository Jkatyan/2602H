#include "main.h"

void opcontrol() {

	PID drive;
	pidInitialize (drive, 0, 0, 0, 127);
	while(1){
		/*
		LD_F.move( controller0.get_analog(ANALOG_LEFT_Y) );
		LD_R.move( controller0.get_analog(ANALOG_LEFT_Y) );
		RD_F.move( controller0.get_analog(ANALOG_RIGHT_Y) );
		RD_R.move( controller0.get_analog(ANALOG_RIGHT_Y) );
		controller0.print(0, 0, "%d    %d    ", (int)(LD_F.get_torque()*1000), (int)(RD_F.get_torque()*1000));
		controller0.print(1, 0, "%d    %d    ", (int)(LD_R.get_torque()*1000), (int)(RD_R.get_torque()*1000));
		delay(200);
		*/

		int atTarget = 0;
		while(atTarget != 1){
			double encoder = TEST_MOTOR.get_position();
			TEST_MOTOR.move(pidCalculate (drive, 1000.0f, encoder));
			if (encoder == 1000){
				atTarget = 1;
				break;
			}
			pros::delay(10);
		}
	}
}
