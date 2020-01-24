#include "main.h"


void opcontrol() {

	while (true) {
		lcd::print(0, "hydra_version: %s", HYDRA_VERSION_STRING);
		delay(20);
	}

}
