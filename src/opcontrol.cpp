#include "main.h"


void opcontrol() {

	testMotor.move_absolute(720, 127);

	while (true) {
		controller0.print(0, 0, "%f", testMotor.get_position() - testMotor.get_target_position());
	}

}
