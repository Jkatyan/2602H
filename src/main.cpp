#include "main.h"
#include "autons.hpp"

void initialize() {
	pros::lcd::initialize();
	LIFT.set_brake_mode(MOTOR_BRAKE_HOLD);
	TILTER.set_brake_mode(MOTOR_BRAKE_HOLD);
	INTAKEA.set_brake_mode(MOTOR_BRAKE_HOLD);
	INTAKEB.set_brake_mode(MOTOR_BRAKE_HOLD);
	pros::ADIGyro gyro (GYROPORT, GC);
	//Custom Filter
	sideEnc.reset();
	leftEnc.reset();
	rightEnc.reset();
	gyro.reset();
  S_reset_all_motors();
	//piston::
	//pros::c::adi_port_set_config(PISTON_L_PORT, pros::E_ADI_DIGITAL_OUT);
//	pros::ADIDigitalOut piston (8);
	//pistonENDS
}
void disabled() {}
void competition_initialize() {}

void autonomous() {
	//TUNE THESE VALUES!!!
	drivePID = pidInit (DRIVEP, DRIVEI, DRIVED, 0, 100.0,5,15);
	turnPID = pidInit (TURNP, TURNI, TURND, 0, 100.0,5,15);
	liftPID = pidInit (LIFTP, LIFTI, LIFTD, 0, 100.0,5,15);
	tilterPID = pidInit (TILTERP, TILTERI, TILTERD, 0, 100.0,5,15);

	while( DISABLE_AUTONOMOUS ){ pros::delay(20); }

	LD.set_brake_mode(MOTOR_BRAKE_HOLD);
	LD2.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD2.set_brake_mode(MOTOR_BRAKE_HOLD);

	//Auton Code Under this line!

	S_chassis_move(-650, 0.5, 5000);
	pros::delay(2500);
	S_chassis_move(950, 0.8, 5000);

	for(;;){pros::delay(20);} //Forever Loop
}


void display_debugInfo(int* d){
	if( *d <= ( 1000 / LCD_DISPLAY_FRAMERATE ) ){
		(*d)++;
	}
	else{
		pros::lcd::print(0, "No debug info");
		/*pros::lcd::print(0, "X: %f", getX());
		pros::lcd::print(1, "Y: %f", getY());
		pros::lcd::print(2, "Angle: %f", getAngleDegrees());
		pros::lcd::print(3, "Gyro Angle: %f", (gyro.get_value()/10.0000));
		pros::lcd::print(4, "Right Encoder: %d", rightEnc.get_value());
		pros::lcd::print(5, "Left Encoder: %d", leftEnc.get_value());
		pros::lcd::print(6, "Side Encoder: %d", sideEnc.get_value());*/
		*d = 0;
	}
}


void opcontrol() {
	int display_update_count = 0;

	LD.set_brake_mode(MOTOR_BRAKE_COAST);
	LD2.set_brake_mode(MOTOR_BRAKE_COAST);
	RD.set_brake_mode(MOTOR_BRAKE_COAST);
	RD2.set_brake_mode(MOTOR_BRAKE_COAST);

	//pros::ADIDigitalOut piston (8);

	while(true){
		while ( /*( !second_controller.get_digital(DIGITAL_R1) ) &&*/ ( !controller.get_digital(DIGITAL_A) ) ) {
			S_drive_chassis_tank();
			S_armsMotion_proceed();

			//pistonTest
			//piston.set_value(controller.get_digital(DIGITAL_A));
			//pistonEnds

			updatePosition();

			display_debugInfo(&display_update_count);

			pros::delay(1);
		}
		S_zero_all_motors();
		while( /*second_controller.get_digital(DIGITAL_R1) ||*/ controller.get_digital(DIGITAL_A) ){
			S_drive_chassis_arcade();
			S_armsMotion_Amode_proceed();

			updatePosition();

			display_debugInfo(&display_update_count);

			pros::delay(1);
		}
		S_zero_all_motors();
	}
}
