#include "main.h"
#include "odometry.hpp"
#include "PID.hpp"

//Motor Ports
const int LDPORT = 17;
const int LD2PORT = 18;
const int RDPORT = 16;
const int RD2PORT = 19;
const int INTAKEAPORT = 5;
const int INTAKEBPORT = 8;
const int LIFTPORT = 6;
const int TILTERPORT = 7;
const char GYROPORT = 'c';

const int PISTON_L_PORT = 1;

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


//backup_autonomous_API
const int AUTOMOVE_ALLOWABLE_ERROR = 36;
const int AUTOMOVE_SUCCESS_HOLDING_TIME = 500;
const int LCD_DISPLAY_FRAMERATE = 30;
//backup_autonomous_API


pros::ADIGyro gyro (GYROPORT, GC);
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


bool S_motorSuccess( pros::Motor motor ){
	int diff = (int) motor.get_position() - motor.get_target_position();

	if( diff > (-1) * AUTOMOVE_ALLOWABLE_ERROR && diff < AUTOMOVE_ALLOWABLE_ERROR ){
		return true;
	}
	else{
		return false;
	}
}


int S_chassis_wait_till_success(int timeOut, int mode){
	uint32_t startTime = pros::millis();
	int successCount = 0;
	int display_update_count = 0;

	while(1){
		if( successCount > AUTOMOVE_SUCCESS_HOLDING_TIME ){
			return 0;
		}
		if( pros::millis() - startTime > timeOut ){
			return -1;
		}
		if( S_motorSuccess(LD) && S_motorSuccess(LD2) && S_motorSuccess(RD) && S_motorSuccess(RD2) ){
			successCount++;
		}
		else{
			successCount = 0;
		}

		if( display_update_count <= ( 1000 / LCD_DISPLAY_FRAMERATE ) ){
			display_update_count++;
		}
		else{
			if(mode == 0){
				pros::lcd::print( 0, "command: move" );
			}
			else{
				pros::lcd::print( 0, "command: turn" );
			}
			pros::lcd::print(1, "target: %f", LD.get_target_position() );
			pros::lcd::print(2, "actual: %f", LD.get_position() );
			pros::lcd::print(3, "success: %d", successCount > 0 );
			pros::lcd::print(4, "successHold: %d", AUTOMOVE_SUCCESS_HOLDING_TIME - successCount );
			pros::lcd::print(5, "timeOut: %d", startTime + timeOut - pros::millis() );
			display_update_count = 0;
		}

		pros::delay(1);
	}
}


int S_chassis_move(int angle, float speed, int timeOut){
	int s = (int) (speed * 127);

	LD.move_relative(angle, s);
	RD.move_relative(angle, s);
	LD2.move_relative(angle, s);
	RD2.move_relative(angle, s);

	return S_chassis_wait_till_success(timeOut, 0);
}


int S_chassis_turn(int angle, float speed, int timeOut){
	int s = (int) (speed * 127);

	LD.move_relative(angle, s);
	RD.move_relative((-1)*angle, s);
	LD2.move_relative(angle, s);
	RD2.move_relative((-1)*angle, s);

	return S_chassis_wait_till_success(timeOut, 1);
}




void autonomous() {

	LD.set_brake_mode(MOTOR_BRAKE_HOLD);
	LD2.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD.set_brake_mode(MOTOR_BRAKE_HOLD);
	RD2.set_brake_mode(MOTOR_BRAKE_HOLD);


drivePID = pidInit (DRIVEP, DRIVEI, DRIVED, 0, 100.0,5,15);
turnPID = pidInit(TURNP, TURNI, TURND, 0, 100.0,5,15);
lastSlewTime = pros::millis();
//Auton Code Under this line!
//goTo(0,20);


S_chassis_move(3600, 0.5, 15000);
S_chassis_turn(1800, 0.5, 15000);
S_chassis_move(-3600, 0.5, 15000);


for(;;){pros::delay(20);} //Forever Loop
}

void opcontrol() {

	LD.set_brake_mode(MOTOR_BRAKE_COAST);
	LD2.set_brake_mode(MOTOR_BRAKE_COAST);
	RD.set_brake_mode(MOTOR_BRAKE_COAST);
	RD2.set_brake_mode(MOTOR_BRAKE_COAST);

	//pros::ADIDigitalOut piston (8);


	while (true) {
		LD.move(LC*(controller.get_analog(ANALOG_LEFT_Y)));
		LD2.move(LC*(controller.get_analog(ANALOG_LEFT_Y)));
		RD.move(RC*(controller.get_analog(ANALOG_RIGHT_Y)));
		RD2.move(RC*(controller.get_analog(ANALOG_RIGHT_Y)));

		//pistonTest
		//piston.set_value(controller.get_digital(DIGITAL_A));
		//pistonEnds

		updatePosition();

		pros::lcd::print(0, "X: %f", getX());
		pros::lcd::print(1, "Y: %f", getY());
		pros::lcd::print(2, "Angle: %f", getAngleDegrees());
		pros::lcd::print(3, "Gyro Angle: %f", (gyro.get_value()/10.0000));
		pros::lcd::print(4, "Right Encoder: %d", rightEnc.get_value());
		pros::lcd::print(5, "Left Encoder: %d", leftEnc.get_value());
		pros::lcd::print(6, "Side Encoder: %d", sideEnc.get_value());

		pros::delay(20);
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
