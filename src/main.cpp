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
		pros::lcd::print(0, "%f", imu1.get_rotation());
		pros::delay(20);
	}

}
