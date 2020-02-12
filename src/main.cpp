#include "main.h"

/*2602H Jscript*/
//Jscript Branch 2602H Github

//PORTS
#define RFPORT 20
#define RBPORT -19
#define LFPORT 11
#define LBPORT -12

#define RC 0.99
#define LC 1

#define DRIVEP 0.4
#define DRIVEI 0
#define DRIVED 0.01

#define TILTERPORT 5
#define LIFTPORT 6
#define RINTPORT 7
#define LINTPORT -8

#define IMUPORT 13

//INITIALIZE
PID drivePID;

float lastSlewTime;
float maxAccel = 0.17; //Chassis
float lastSlewRate = 0;

pros::Imu imu(IMUPORT); //Pros Motors

pros::Motor RD(abs(RFPORT));
pros::Motor RD2(abs(RBPORT), true);
pros::Motor LD(abs(LFPORT));
pros::Motor LD2(abs(LBPORT), true);

pros::Motor TILTER(abs(TILTERPORT));
pros::Motor LIFT(abs(LIFTPORT));
pros::Motor RIGHTINTAKE(abs(RINTPORT));
pros::Motor LEFTINTAKE(abs(LINTPORT), true);

using namespace okapi;

Motor rf(RFPORT); //Okapi Motors
Motor rb(RBPORT);
Motor lf(LFPORT);
Motor lb(LBPORT);

Motor tilter(TILTERPORT);
Motor lift(LIFTPORT);
Motor rintake(RINTPORT);
Motor lintake(LINTPORT);

MotorGroup intake({RINTPORT, LINTPORT});

Controller master;

//OKAPI CONTROLLERS

auto liftController = AsyncPosControllerBuilder()
                        .withMotor(LIFTPORT) // lift motor port 3
                        //.withGains({liftkP, liftkI, liftkD})
                        .build();
auto myChassis =
  ChassisControllerBuilder()
    .withMotors({LFPORT, LBPORT}, {RFPORT, RBPORT})
    // Green Gearset, 3.25 in wheel diam, 10 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10_in}, imev5GreenTPR})
    .build();

auto tilterController = AsyncPosControllerBuilder()
                        .withMotor(TILTERPORT) // lift motor port 3
                        //.withGains({tilterkP, tilterkI, tilterkD})
                        .build();

auto profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      0.5,  // Maximum linear velocity of the Chassis in m/s
      2,  // Maximum linear acceleration of the Chassis in m/s/s
      2 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(myChassis)
    .buildMotionProfileController();

    void initialize() {
      imu.reset();
      pros::delay(2000);
    	pros::lcd::initialize();
    	pros::lcd::set_text(1, "2602H JScript");
    }

    void disabled() {}
    void competition_initialize() {
      profileController->generatePath(
        {{1_ft, 0_ft, 0_deg}, {3_ft, -0.7_ft, 0_deg}}, "B");
    }

//DRIVE FUNCTIONS
void setDriveBrakes(pros::motor_brake_mode_e_t mode){
  LD.set_brake_mode(mode);
  LD2.set_brake_mode(mode);
  RD.set_brake_mode(mode);
  RD2.set_brake_mode(mode);
}

void setDrive(int left, int right){
  LD.move(left);
  LD2.move(left);
  RD.move(right);
  RD2.move(right);
}

void resetDrive(){
  LD.tare_position();
  LD2.tare_position();
  RD.tare_position();
  RD2.tare_position();
  setDrive(0,0);
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

void driveTarget(int target, int time, float speed){
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();

    while ((atTarget != 1) && (pros::millis()-startTime) < time) {
    driveEnc = RD.get_position();
    distance = target - driveEnc;

		//pros::lcd::print(1, "Drive Encoder Value: %f", driveEnc);
		//pros::lcd::print(2, "Distance from Target: %f", distance);
    //pros::lcd::print(4, "Encoder: %d", RD.get_position());

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
       //pros::lcd::print(2, "Distance from Target: %f", distance);
      }
      pros::delay(20);
    }
}

void rotate(int degrees, int voltage) {
int direction = abs(degrees)/degrees;
setDrive(-voltage * direction, voltage * direction);
while(fabs(imu.get_yaw()) < abs(degrees) - 0.5){
  pros::delay(10);
}
pros::delay(100);
if(fabs(imu.get_yaw()) > abs(degrees)){
  setDrive(0.5 * voltage * direction, 0.5 * -voltage * direction);
  while(fabs(imu.get_yaw()) > abs(degrees)){
    pros::delay(10);
  }
}
else if(fabs(imu.get_yaw()) < abs(degrees)){
  setDrive(0.5 * -voltage * direction, 0.5 * voltage * direction);
  while(fabs(imu.get_yaw()) > abs(degrees)){
    pros::delay(10);
  }
}
resetDrive();
}

//MAIN (AUTON) CODE
void autonomous(){
  //AUTON INIT
  drivePID = pidInit (DRIVEP, DRIVEI, DRIVED, 0, 100.0,5,15);
  setDriveBrakes(MOTOR_BRAKE_HOLD);
  lift.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(AbstractMotor::brakeMode::brake);
  resetDrive();
  //AUTON START
  rotate(-90,60);
/*
  //Flipout
  //Intake max speed in
  driveTarget(600,2000, 1);
  driveTarget(200,2000, 1);
  driveTarget(2000,5000, 0.6);
  //Raise arms
  driveTarget(2300 ,1000, 1);
  //Arms down
  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2_ft, 0.9_ft, 0_deg}}, "A");
  profileController->setTarget("A", true, true); //setTarget("A", true, true); to follow path backwards.
              profileController->waitUntilSettled();
              profileController->removePath("A"); //remove path once motion is complete.
  resetDrive();
  driveTarget(-500,5000, 0.6);
  driveTarget(2200,5000, 0.6);
  driveTarget(800,5000, 0.6);
  //Turn to face zone
  //Drive forward
  //Deposit
  //Drive back
*/
}
void opcontrol() {
  setDriveBrakes(MOTOR_BRAKE_COAST);
  lift.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(AbstractMotor::brakeMode::brake);
  while(true){
    pros::lcd::print(1, "IMU Heading: %f", imu.get_heading());
    pros::lcd::print(2, "IMU Rotations: %f", imu.get_rotation());
    pros::lcd::print(3, "IMU Drift: %f", imu.get_gyro_rate());
    pros::lcd::print(4, "IMU Pitch: %f", imu.get_pitch());
    pros::lcd::print(5, "IMU Roll: %f", imu.get_roll());
    pros::lcd::print(6, "IMU Yaw: %f", imu.get_yaw());
    myChassis->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));
    pros::delay(20);
  }
}
