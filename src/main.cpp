#include "main.h"
#include "autons.hpp"

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

	LD.set_brake_mode(MOTOR_BRAKE_HOLD);
	LD2.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD2.set_brake_mode(MOTOR_BRAKE_HOLD);

	//Auton Code Under this line!
	drive.setMaxVelocity(100);
	//A_driveTarget(1000, 100000, 1);
	INTAKEA.move(127);
	INTAKEB.move(127);
	drive.moveDistance(0.5_ft);
	A_motorTarget(TILTERPORT, tilterPID, 1, 1280, 500, 1 , 0.03, false);
	drive.moveDistanceAsync(4_ft);
	drive.waitUntilSettled();
	drive.moveDistanceAsync(-4_ft);
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
	LD.set_brake_mode(MOTOR_BRAKE_COAST);
	LD2.set_brake_mode(MOTOR_BRAKE_COAST);
	RD.set_brake_mode(MOTOR_BRAKE_COAST);
	RD2.set_brake_mode(MOTOR_BRAKE_COAST);
	Controller masterController;
	tilterPID = pidInit (TILTERP, TILTERI, TILTERD, 0, 100.0,5,15);
		while (true){
			if(controller.get_digital(DIGITAL_L1)){
				LIFT.move(-127);
			} else 	if(controller.get_digital(DIGITAL_L2)){
				LIFT.move(127);
			}
			else {
				LIFT.move(0);
				liftController.stop();
			}
			S_moveMotor_withLimit(INTAKEA, INTAKEA_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
			S_moveMotor_withLimit(INTAKEB, INTAKEB_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
			S_moveMotor_withLimit(TILTER, TILTER_SPEED, 1280, 3650, DIGITAL_A, DIGITAL_B, 2);
			drive.tank(masterController.getAnalog(ControllerAnalog::leftY), masterController.getAnalog(ControllerAnalog::rightY));

			if(controller.get_digital(DIGITAL_X)){ //Bring Up Stack
				INTAKEA.move(-5);
				INTAKEB.move(-5);
				LD.set_brake_mode(MOTOR_BRAKE_HOLD);
				LD2.set_brake_mode(MOTOR_BRAKE_HOLD);
				RD.set_brake_mode(MOTOR_BRAKE_HOLD);
				RD2.set_brake_mode(MOTOR_BRAKE_HOLD);
				A_motorTarget(TILTERPORT, tilterPID, 1, 3650, 4000, 0.7, 0.02, false);
				pros::delay(20);
				drive.stop();
			}
			else
			{
				LD.set_brake_mode(MOTOR_BRAKE_COAST);
				LD2.set_brake_mode(MOTOR_BRAKE_COAST);
				RD.set_brake_mode(MOTOR_BRAKE_COAST);
				RD2.set_brake_mode(MOTOR_BRAKE_COAST);
			}
			/*
			-If driving backwards and tray is > 90, intake out 
			-Add button to bring down tray
			*/
			if(controller.get_digital(DIGITAL_UP)){
				macroTrueArmHigh = 1;
			}
			if(controller.get_digital(DIGITAL_RIGHT)){
				macroTrueArmMid = 1;
			}
			if(controller.get_digital(DIGITAL_DOWN)){
				macroTrueArmLow = 1;
			}

			if(macroTrueArmLow == 1){
				liftController.setTarget(0);
				macroTrueArmLow = 0;
			}
			if(macroTrueArmMid == 1){
				liftController.setTarget(-2100);
				macroTrueArmMid = 0;
			}
			if(macroTrueArmHigh == 1){
				liftController.setTarget(-3000);
				macroTrueArmHigh = 0;
			}


			updatePosition();
			display_debugInfo(&display_update_count);

			pros::delay(10);
		}
		S_zero_all_motors();
}
