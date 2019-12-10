#include "main.h"
#include "odometry.hpp"
#include "PID.hpp"
#include "autonLib\autons.hpp"


//Motor Ports
const int LDPORT = 17;
const int LD2PORT = 18;
const int RDPORT = 16;
const int RD2PORT = 19;
const int INTAKEAPORT = 5;
const int INTAKEBPORT = 9;
const int LIFTPORT = 6;
const int TILTERPORT = 7;
const char GYROPORT = 'c';
const char POTPORT = 'h';
//const char PISTON_L_PORT = 'a';

//PID Tuning
const float DRIVEP = 7;
const float DRIVEI = 0;
const float DRIVED = 0;
const float TURNP = 40;
const float TURNI = 0;
const float TURND = 0;

//Correction Constants
const double RC = 1; //Chassis Speed Correction
const double LC = 1;
const double GC = 0.95; //Gyro Correction


//S_armsMotion_proceed{
const int TILTER_MAX_VALUE = 0;
const int TILTER_MIN_VALUE = -2950;
const int TILTER_SPEED = 127;
const int LIFT_MAX_VALUE = 1300;
const int LIFT_MIN_VALUE = 0;
const int LIFT_SPEED = 127;
const int INTAKEA_SPEED = 127;
const int INTAKEB_SPEED = 127;
//}

//backup_autonomous_API
const int AUTOMOVE_ALLOWABLE_ERROR = 36;
const int AUTOMOVE_SUCCESS_HOLDING_TIME = 500;
const int LCD_DISPLAY_FRAMERATE = 30;
//backup_autonomous_API

const bool DISABLE_AUTONOMOUS = false;


pros::ADIGyro gyro (GYROPORT, GC);
pros::ADIAnalogIn pot (POTPORT);

pros::ADIEncoder sideEnc('G', 'H', true);
pros::ADIEncoder leftEnc('F', 'E');
pros::ADIEncoder rightEnc('A', 'B', true);

PID drivePID;
PID turnPID;

float lastSlewTime;
float maxAccel = 0.14;
float lastSlewRate = 0;

void goTo(float targetX, float targetY);
void setDrive(int left, int right);
void rotate(int degrees, int voltage);
float slewRateCalculate(float desiredRate);
void driveTarget(int target, int time, float speed);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Controller second_controller(pros::E_CONTROLLER_PARTNER);
pros::Motor LD(LDPORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor LD2(LD2PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RD(RDPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RD2(RD2PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor INTAKEA(INTAKEAPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor INTAKEB(INTAKEBPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor TILTER(TILTERPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor LIFT(LIFTPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);

auto liftController = AsyncControllerFactory::posPID(LIFTPORT, 0.001, 0.0, 0.0001);
auto intakeController = AsyncControllerFactory::posPID({INTAKEAPORT, -INTAKEBPORT}, 0.001, 0.0, 0.0001);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	LIFT.set_brake_mode(MOTOR_BRAKE_HOLD);
	TILTER.set_brake_mode(MOTOR_BRAKE_HOLD);
	INTAKEA.set_brake_mode(MOTOR_BRAKE_HOLD);
	INTAKEB.set_brake_mode(MOTOR_BRAKE_HOLD);
	pros::ADIGyro gyro (GYROPORT, GC);
	pros::delay(2000);
	sideEnc.reset();
	leftEnc.reset();
	rightEnc.reset();
	gyro.reset();
	LD.tare_position();
	LD2.tare_position();
	RD.tare_position();
	RD2.tare_position();
	LIFT.tare_position();
	INTAKEA.tare_position();
	INTAKEB.tare_position();
	TILTER.tare_position();
	//piston::
	//pros::c::adi_port_set_config(PISTON_L_PORT, pros::E_ADI_DIGITAL_OUT);
//	pros::ADIDigitalOut piston (8);
	//pistonENDS
}
void disabled() {}
void competition_initialize() {}

/*
How to use Okapi
  liftController.setTarget(200); // Move 200 motor degrees upward
  liftController.waitUntilSettled(); //Wait until lift settled

Auton Functions
	driveTarget - Param: Target (Encoder Counts), Time Allowed (MS), Speed (0-1). Ex. driveTarget(1000, 2500, 1);
	goTo - Param: X,Y coordinates of target point. Ex. goTo(1,4);
	rotate - Param: Degrees, Voltage. Ex. rotate(90,127);
*/


void autonomous() {
	while( DISABLE_AUTONOMOUS ){ pros::delay(20); }

	LD.set_brake_mode(MOTOR_BRAKE_HOLD);
	LD2.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD2.set_brake_mode(MOTOR_BRAKE_HOLD);

	drivePID = pidInit (DRIVEP, DRIVEI, DRIVED, 0, 100.0,5,15);
	turnPID = pidInit(TURNP, TURNI, TURND, 0, 100.0,5,15);
	lastSlewTime = pros::millis();

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

void goTo(float targetX, float targetY) {
	bool atPoint = false;
	float targetAngle =0;
	float power =0;
	float turnPower =0;
	lastSlewTime = pros::millis();

	while (!atPoint) {
		updatePosition();

		power = -pidCalculate(drivePID, 0, sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)));
		power = slewRateCalculate(power);

		targetAngle = atan2f((targetY-getY()),(targetX-getX()))-M_PI/2;
		if (targetAngle >= M_PI) {
	    targetAngle-=2*M_PI;
	  }
	  else if (targetAngle <= -M_PI) {
	    targetAngle+=2*M_PI;
	  }

		turnPower = ((fabs(targetAngle-getAngle())>M_PI)? -1: 1)*pidCalculate(turnPID, targetAngle, getAngle());

		LD.move((power + turnPower)*LC);
    RD.move((power - turnPower)*RC);
		LD2.move((power + turnPower)*LC);
		RD2.move((power - turnPower)*RC);

		pros::lcd::print(0, "X: %f", getX());
		pros::lcd::print(1, "Y: %f", getY());
		pros::lcd::print(2, "Angle: %f", getAngleDegrees());

		pros::lcd::print(7, "Target Angle: %f", targetAngle*180/M_PI);


		if (sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)) < 3 || controller.get_digital(DIGITAL_B)) {
			atPoint = true;
		}
		pros::delay(10);
	}
}

float slewRateCalculate (float desiredRate) {
		//pros::lcd::print(7, "called: %f", desiredRate);
		float deltaTime = pros::millis()-lastSlewTime;
		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
		float addedRate;
		float newRate;

		if (fabs(desiredAccel) < maxAccel || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
		    addedRate = desiredAccel*deltaTime;
		    newRate = addedRate+lastSlewRate;
		}
		else {
		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccel*deltaTime;
        newRate = addedRate+lastSlewRate;
		}
	  lastSlewTime = lastSlewTime+deltaTime;
	  lastSlewRate = newRate;

		float returnVal = newRate;
		return returnVal;
}

void setDrive(int left, int right){
	LD = left;
	LD2 = left;
	RD = right;
	RD2 = right;
}

void rotate(int degrees, int voltage){
	int direction = abs(degrees) / degrees;
	//gyro.reset();
	setDrive(-voltage * direction, voltage * direction);
	while(fabs(gyro.get_value()) < abs(degrees * 10) - 50){
		pros::delay(10);
	}
	pros::delay(100);
	if(fabs(gyro.get_value()) > abs(degrees * 10)){
		setDrive(0.5 * voltage * direction, 0.5 * -voltage * direction);
		while(fabs(gyro.get_value()) > abs(degrees * 10)){
			pros::delay(10);
		}
	}
	else if(fabs(gyro.get_value()) < abs(degrees * 10)){
		setDrive(0.5 * -voltage * direction, 0.5 * voltage * direction);
		while(fabs(gyro.get_value()) > abs(degrees * 10)){
			pros::delay(10);
		}
	}
	setDrive(0,0);
}

void driveTarget(int target, int time, float speed){
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();

    while ((atTarget != 1) && (pros::millis()-startTime) < time) {
    driveEnc = ((abs(leftEnc.get_value()))+(abs(rightEnc.get_value())))/2;
    distance = target - driveEnc;

		pros::lcd::print(1, "Drive Encoder Value: %f", driveEnc);
		pros::lcd::print(2, "Distance from Target: %f", distance);

    float val = pidCalculate(drivePID, target, driveEnc)*speed;
    val = slewRateCalculate(val);
    int rightVal = val;
    int leftVal = val;

    RD.move(RC*rightVal);
    RD2.move(RC*rightVal);
    LD.move(LC*leftVal);
    LD2.move(LC*leftVal);
    if(driveEnc == target){
       atTarget = 1;
       pros::lcd::print(2, "Distance from Target: %f", distance);
      }
      pros::delay(20);
    }
}
