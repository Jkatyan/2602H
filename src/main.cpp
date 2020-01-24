#include "main.h"


void initialize() {
	lcd::initialize();
}


void disabled(){;}


void competition_initialize(){;}


void autonomous(){
	//autonMain();
}


void opcontrol() {

	Imu imu1(7);


	while (true) {
		lcd::print(0, "%f", imu1.get_rotation());
		lcd::print(0, "%d", co_co);
		delay(20);
	}

}
