#include "main.h"


void opcontrol() {

	TEST_MOTOR.move_absolute(720, 127);

	while (true) {
		controller0.print(0, 0, "%f", TEST_MOTOR.get_position() - TEST_MOTOR.get_target_position());
	}

}
