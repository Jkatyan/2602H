#include "main.h"
#include "odometry.hpp"
#include "PID.hpp"

pros::ADIGyro gyro ('b', 1);
pros::ADIEncoder sideEnc('G', 'H');

const int LDPORT = 1;
const int LD2PORT = 2;
const int RDPORT = 3;
const int RD2PORT = 4;

const int RIGHTLIFTPORT = 5;
const int LEFTLIFTPORT = 8;
const int CHASSISPORT = 6;
const int HOOKPORT = 7;

const int RC = 1; //Chassis Speed Correction
const int LC = 1;

PID drivePID;
PID turnPID;

float lastSlewTime;
float maxAccel = 0.14;
float lastSlewRate = 0;

void goTo(float targetX, float targetY);
void setDrive(int left, int right);
void rotate(int degrees, int voltage);
float slewRateCalculate(float desiredRate);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor LD(LDPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor LD2(LD2PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RD(RDPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RD2(RD2PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RIGHTLIFT(RIGHTLIFTPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor LEFTLIFT(LEFTLIFTPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor CHASSISEXTENSION(CHASSISPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor INTAKE(HOOKPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);

auto liftController = AsyncControllerFactory::posPID({-RIGHTLIFTPORT, LEFTLIFTPORT}, 0.001, 0.0, 0.0001);
auto filpController = AsyncControllerFactory::posPID(CHASSISPORT, 0.001, 0.0, 0.0001);
auto intakeController = AsyncControllerFactory::posPID(HOOKPORT, 0.001, 0.0, 0.0001);

/*
  liftController.setTarget(200); // Move 200 motor degrees upward
  liftController.waitUntilSettled(); //Wait until lift settled
*/

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
	LEFTLIFT.set_brake_mode(MOTOR_BRAKE_HOLD);
	RIGHTLIFT.set_brake_mode(MOTOR_BRAKE_HOLD);
	CHASSISEXTENSION.set_brake_mode(MOTOR_BRAKE_HOLD);
	INTAKE.set_brake_mode(MOTOR_BRAKE_HOLD);
	pros::ADIGyro gyro ('b', 1);
	pros::delay(2000);
}
void disabled() {}
void competition_initialize() {}

void autonomous() {
	goTo(0, 10);
  liftController.setTarget(200);
	rotate(180, 127);
  goTo(0, 5);
}
void opcontrol() {

	drivePID = pidInit (7, 0, 0, 0, 100.0,5,15);
	turnPID = pidInit(40, 0, 0, 0, 100.0,5,15);
	lastSlewTime = pros::millis();

	while (true) {
		LD.move(LC*(controller.get_analog(ANALOG_LEFT_Y)));
		LD2.move(LC*(controller.get_analog(ANALOG_LEFT_Y)));
		RD.move(RC*(controller.get_analog(ANALOG_RIGHT_Y)));
		RD2.move(RC*(controller.get_analog(ANALOG_RIGHT_Y)));

		/*if (controller.get_digital_new_press(DIGITAL_X)) {
			moveToPoint(0, 0);
		}*/
		updatePosition();

		pros::lcd::print(0, "X: %f", getX());
		pros::lcd::print(1, "Y: %f", getY());
		pros::lcd::print(2, "Angle: %f", getAngleDegrees());

		pros::lcd::print(4, "Side Encoder: %d", sideEnc.get_value());
		pros::lcd::print(5, "Left Encoder: %f", LD.get_position());
		pros::lcd::print(6, "Right Encoder: %f", RD.get_position());

		pros::delay(10);
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
	gyro.reset();
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
