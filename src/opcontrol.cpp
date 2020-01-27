#include "main.h"


void opcontrol() {

	TEST_MOTOR.move_absolute(720, 127);

	while (true) {
		controller0.print(0, 0, "Line1 code");
		controller0.print(1, 0, "Line2 code");
		controller0.print(2, 0, "Line3 code");
	}

}
