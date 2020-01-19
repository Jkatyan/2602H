#include "main.h"
#include "autons.hpp"

void setDriveBrakes(pros::motor_brake_mode_e_t mode){
  LD.set_brake_mode(mode);
  LD2.set_brake_mode(mode);
  RD.set_brake_mode(mode);
  RD2.set_brake_mode(mode);
}

void initialize() {
	pros::lcd::initialize();
	LIFT.set_brake_mode(MOTOR_BRAKE_HOLD);
	TILTER.set_brake_mode(MOTOR_BRAKE_HOLD);
	INTAKEA.set_brake_mode(MOTOR_BRAKE_HOLD);
	INTAKEB.set_brake_mode(MOTOR_BRAKE_HOLD);
	pros::ADIGyro gyro (GYROPORT, GC);
	sideEnc.reset();
	leftEnc.reset();
	rightEnc.reset();
	gyro.reset();
  S_reset_all_motors();
}
void disabled() {}
void competition_initialize() {}

using namespace okapi;
int OLD = LDPORT;
int OLD2 = LD2PORT * -1;
int ORD = RDPORT * -1;
int ORD2 = RD2PORT;
int OLIFT = LIFTPORT;
auto drive = ChassisControllerFactory::create(
	{OLD2,OLD},
	{-ORD2,-ORD},
	AbstractMotor::gearset::green,
	{3.25_in, 10_in}
);
auto liftController = AsyncControllerFactory::posIntegrated(OLIFT);

void autonomous() {
	//TUNE THESE VALUES!!
	drivePID = pidInit (DRIVEP, DRIVEI, DRIVED, 0, 100.0,5,15);
	turnPID = pidInit (TURNP, TURNI, TURND, 0, 100.0,5,15);
	liftPID = pidInit (LIFTP, LIFTI, LIFTD, 0, 100.0,5,15);
	tilterPID = pidInit (TILTERP, TILTERI, TILTERD, 0, 100.0,5,15);

	while( DISABLE_AUTONOMOUS ){ pros::delay(20); }

	setDriveBrakes(MOTOR_BRAKE_HOLD);

	//Auton Code Under this line!
	A_motorTarget(TILTERPORT, tilterPID, 1, 1280, 500, 1, 0.02, false);
		LIFT.move(-127);
		pros::delay(500);
		LIFT.move(127);
		pros::delay(500);
		LIFT.move(0);
		A_driveTarget(200, 3000, 0.6);
		drive.stop();

	/*
	drive.setMaxVelocity(100);
	INTAKEA.move(127);
	INTAKEB.move(127);
A_driveTarget(1200, 3000, 0.6);
A_driveTargetBack(400, 2000, 0.6);
S_chassis_turn(200, 0.4, 3000);
pros::delay(500);
S_reset_all_motors();
INTAKEA.move(-25);
INTAKEB.move(-25);
drive.moveDistanceAsync(0.9_ft);
drive.waitUntilSettled();
A_motorTarget(TILTERPORT, tilterPID, 1, 3600, 3000, 1, 0.02, false);

drive.setMaxVelocity(100);
INTAKEA.move(-50);
INTAKEB.move(-50);
drive.moveDistanceAsync(-0.5_ft);
A_motorTarget(TILTERPORT, tilterPID, 1, 1280, 2000, 1, 0.02, false);
drive.waitUntilSettled();
drive.stop();
*/
/*S_chassis_turn(55, 0.4, 1000);
S_reset_all_motors();
A_driveTarget(1000, 2000, 0.6);
A_driveTargetBack(0, 2000, 0.6);
S_chassis_turn(120, 0.4, 2000);*/

	//A_driveTarget(1000, 100000, 1);
/*
	drive.moveDistance(0.5_ft);
	A_motorTarget(TILTERPORT, tilterPID, 1, 1280, 500, 1 , 0.03, false);
	drive.moveDistanceAsync(4_ft);
	drive.waitUntilSettled();
	drive.moveDistanceAsync(-4_ft);*/
	/*S_chassis_move(2000, 0.5, 2500);
	A_gyroTurn(-90,1,5000,1);
	S_chassis_move(2000, 0.5, 2500);
	S_chassis_move(-2000, 0.5, 2500);
  A_gyroTurn(-90,1,5000,1);
	S_chassis_move(-2000, 0.5, 2500);*/

	for(;;){pros::delay(20);} //Forever Loop
}


void display_debugInfo(int* d){
	if( *d <= ( 1000 / LCD_DISPLAY_FRAMERATE ) ){
		(*d)++;
	}
	else{
		//pros::lcd::print(0, "No debug info");
		//pros::lcd::print(0, "X: %f", getX());
		//pros::lcd::print(1, "Y: %f", getY());
		//pros::lcd::print(2, "Angle: %f", getAngleDegrees());
		pros::lcd::print(3, "Gyro Angle: %f", (gyro.get_value()/10.0000));
		pros::lcd::print(4, "Right Encoder: %d", RD.get_position());
		pros::lcd::print(5, "Left Encoder: %d", LD.get_position());
		//pros::lcd::print(6, "Side Encoder: %d", sideEnc.get_value());
		*d = 0;
	}
}


void opcontrol() {
	int display_update_count = 0;
	int macroTrueArmHigh = 0; int macroTrueArmLow = 0; int macroTrueArmMid = 0; int macroArm = 0;
	setDriveBrakes(MOTOR_BRAKE_COAST);
	Controller masterController;
	tilterPID = pidInit (TILTERP, TILTERI, TILTERD, 0, 100.0,5,15);
		while (true){
			drive.setMaxVelocity(200);
			if(controller.get_digital(DIGITAL_L1)){
				LIFT.move(-127);
			} else 	if(controller.get_digital(DIGITAL_L2)){

				LIFT.move(127);

			}
			else {
				LIFT.move(0);
				liftController.stop();
			}
			if(controller.get_digital(DIGITAL_UP)){
				TILTER.move(-127);
			} else if(controller.get_digital(DIGITAL_DOWN)){
				if(pot.get_value() >= 1280){
				TILTER.move(127);
			}
			}
			else {
				TILTER.move(0);
			}
			S_moveMotor_withLimit(INTAKEA, INTAKEA_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
			S_moveMotor_withLimit(INTAKEB, INTAKEB_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
			drive.tank(masterController.getAnalog(ControllerAnalog::leftY), masterController.getAnalog(ControllerAnalog::rightY));

			if(controller.get_digital(DIGITAL_X)){ //Bring Up Stack
				drive.setMaxVelocity(200);
				INTAKEA.move(-5);
				INTAKEB.move(-5);
				setDriveBrakes(MOTOR_BRAKE_HOLD);
				//A_motorTarget(TILTERPORT, tilterPID, 1, 2465, 1500, 1, 0.02, false);
				A_motorTarget(TILTERPORT, tilterPID, 1, 3650, 3500, 0.6, 0.02, false);
				pros::delay(20);
				drive.stop();
			}
			else if(controller.get_digital(DIGITAL_Y)){ //Bring Up Stack
					drive.setMaxVelocity(30);
					drive.moveDistanceAsync(-0.4_ft);
					INTAKEA.move(-45);
					INTAKEB.move(-45);
				  setDriveBrakes(MOTOR_BRAKE_COAST);
					A_motorTarget(TILTERPORT, tilterPID, 1, 2465, 1000, 0.7, 0.02, false);
					A_motorTarget(TILTERPORT, tilterPID, 1, 1280, 1000, 1, 0.02, false);
					drive.stop();
				}
			else
			{
				drive.setMaxVelocity(200);
				setDriveBrakes(MOTOR_BRAKE_COAST);
			}

			if(controller.get_digital(DIGITAL_LEFT)){
				A_motorTarget(TILTERPORT, tilterPID, 1, 1280, 1000, 1, 0.02, false);
			}

			if(controller.get_digital(DIGITAL_RIGHT)){
				macroTrueArmLow = 1;
			}

			if(macroTrueArmLow == 1){
				liftController.setTarget(0);
				macroTrueArmLow = 0;
			}

			//updatePosition();
			//display_debugInfo(&display_update_count);

			pros::delay(20);
		}
		S_zero_all_motors();
}
