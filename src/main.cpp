#include "main.h"

/*2602H Jscript*/
//Jscript Branch 2602H Github

//PORTS
#define RFPORT 20
#define RBPORT 19
#define LFPORT 11
#define LBPORT 12

#define TILTERPORT -9
#define LIFTPORT 3
#define RINTPORT 1
#define LINTPORT -10

#define IMUPORT 13
#define IMUPORTB 18
#define POTPORT 'c'

//SETTINGS
#define DRIVEP 0.4
#define DRIVEI 0
#define DRIVED 0.01

#define GYROP 5.2
#define GYROI 0.1
#define GYROD 0.38

#define TILTERP 0.18
#define TILTERI 0
#define TILTERD 0

#define RC 1 //Right Chassis Speed
#define LC 1 //Left Chassis Speed

#define INTAKE_IN 127
#define INTAKE_OUT -127

#define LIFT_UP -127
#define LIFT_DOWN 100

#define TILTER_SPEED 127
#define TILTER_MIN 1054 //Tilter Pot Min Position
#define TILTER_MAX 3380 //Tilter Pot Max Position

//INITIALIZE
PID drivePID;
PID gyroPID;

float lastSlewTime;
float maxAccel = 0.17; //Chassis
float lastSlewRate = 0;

pros::Imu imu(IMUPORT);
pros::Imu imuB(IMUPORTB);
pros::ADIAnalogIn pot (POTPORT);

pros::ADIUltrasonic leftUltra ('E'/*Orange*/, 'F'/*Yellow*/);
pros::ADIUltrasonic rightUltra ('G'/*Orange*/, 'H'/*Yellow*/);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor RD(abs(RFPORT)); //Pros Motors
pros::Motor RD2(abs(RBPORT), true);
pros::Motor LD(abs(LFPORT));
pros::Motor LD2(abs(LBPORT), true);

pros::Motor TILTER(abs(TILTERPORT), true);
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

void initializePID(){
  drivePID = pidInit (DRIVEP, DRIVEI, DRIVED, 0, 100.0,5,15);
  gyroPID = pidInit (GYROP, GYROI, GYROD, 0, 10.0,99999,99999);
  tilterPID = pidInit (TILTERP, TILTERI, TILTERD, 0, 100.0,5,15);
}

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
      imuB.reset();
      pros::delay(2000);
    	pros::lcd::initialize();
    	pros::lcd::set_text(1, "2602H JScript");
    }

    void disabled() {}
    void competition_initialize() {
    }

//DRIVE FUNCTIONS
void setDriveBrakes(pros::motor_brake_mode_e_t mode){
  LD.set_brake_mode(mode);
  LD2.set_brake_mode(mode);
  RD.set_brake_mode(mode);
  RD2.set_brake_mode(mode);
}

void setArmBrakes(pros::motor_brake_mode_e_t mode){
  RIGHTINTAKE.set_brake_mode(mode);
  LEFTINTAKE.set_brake_mode(mode);
  LIFT.set_brake_mode(mode);
  TILTER.set_brake_mode(mode);
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
      }
      pros::delay(20);
    }
}

float motorSlew (float desiredRate, float maxAccelMotor) {
    		float deltaTime = pros::millis()-lastSlewTime;
    		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
    		float addedRate;
    		float newRate;

    		if (fabs(desiredAccel) < maxAccelMotor || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
    		    addedRate = desiredAccel*deltaTime;
    		    newRate = addedRate+lastSlewRate;
    		}
    		else {
    		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccelMotor*deltaTime;
            newRate = addedRate+lastSlewRate;
    		}
    	  lastSlewTime = lastSlewTime+deltaTime;
    	  lastSlewRate = newRate;

    		float returnVal = newRate;
    		return returnVal;
    }

void motorTarget(int port, PID pid, int special, int target, int time, float speed, float accel, bool slew){
  pros::Motor motor(port);
  int atTarget = 0;
  int encoder = 0;
  int startTime = pros::millis();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
  switch(special){case 0:encoder = motor.get_position();break;case 1:encoder = pot.get_value();break;default:break;}
  float val = pidCalculate(pid, target, encoder)*speed;
  val = (slew)? motorSlew(val, accel): val;
  motor.move(val);
  if(encoder == target){
    atTarget = 1;
  }
  pros::delay(15);
  }
  motor.move(0);
}

void rotate(int target, int time, float speed){
//  int direction = abs(target)/target;
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();

  while ((atTarget != 1) && (pros::millis()-startTime) < time) {
  driveEnc = (imu.get_heading() + imuB.get_heading())/2;
  distance = target - driveEnc;

  float val = pidCalculate(gyroPID, target, driveEnc)*speed;
  val = ((fabs(driveEnc-target)>180)? -1 : 1)*val;
  val = slewRateCalculate(val);

    RD.move(-RC*val);
    RD2.move(-RC*val);
    LD.move(LC*val);
    LD2.move(LC*val);

  if(driveEnc == target){
     atTarget = 1;
     //pros::lcd::print(2, "Distance from Target: %f", distance);
    }
    pros::delay(20);
  }
  resetDrive();
}

//MAIN (AUTON) CODE
void autonomous(){
  initializePID();
  setDriveBrakes(MOTOR_BRAKE_HOLD);
  setArmBrakes(MOTOR_BRAKE_HOLD);
  lift.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(AbstractMotor::brakeMode::hold);
  resetDrive();

  /*
  LEFTINTAKE.move(30);

  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2_ft, 2_ft, 90_deg}}, "A");

  LEFTINTAKE.move(0);


  profileController->setTarget("A");
  profileController->waitUntilSettled();
  profileController->removePath("A");

  //AUTON START
 //rotate(270,2000,0.5);
*/
/*
  while(true){
    pros::lcd::print(2, "IMU 1 Drift: %f", imu.get_gyro_rate());
    pros::lcd::print(3, "IMU 1 Yaw: %f", imu.get_yaw());
    pros::lcd::print(5, "IMU 2 Drift: %f", imuB.get_gyro_rate());
    pros::lcd::print(6, "IMU 2 Yaw: %f", imuB.get_yaw());
    pros::delay(20);
  }
*/

  //Flipout
  //Intake max speed in
  driveTarget(600,2000, 1);
  driveTarget(200,2000, 1);
  driveTarget(2000,4000, 0.8);
  //Raise arms
  driveTarget(2300 ,1000, 1);
  //Arms down
  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2_ft, 0.9_ft, 0_deg}}, "A");
  profileController->setTarget("A", true, true); //setTarget("A", true, true); to follow path backwards.
              profileController->waitUntilSettled();
              profileController->removePath("A"); //remove path once motion is complete.
  resetDrive();
  driveTarget(-500,1000, 0.8);
  driveTarget(2000,4000, 0.8);
  driveTarget(800,3500, 0.8);
  //Turn to face zone
  rotate(-137,60, 1.0);
  resetDrive();
  driveTarget(950,3500, 0.8);
  //Drive forward
  //Deposit
  //Drive back

}
void opcontrol() {
  initializePID();
  setDriveBrakes(MOTOR_BRAKE_COAST);
  setArmBrakes(MOTOR_BRAKE_HOLD);
  lift.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(AbstractMotor::brakeMode::hold);
  while(true){
    pros::lcd::print(2, "IMU 1 Yaw: %f", imu.get_yaw());
    pros::lcd::print(3, "IMU 2 Yaw: %f", imuB.get_yaw());
    myChassis->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));
    //LIFT
    (controller.get_digital(DIGITAL_L1)) ? LIFT.move(LIFT_UP) : (controller.get_digital(DIGITAL_L2)) ? LIFT.move(LIFT_DOWN) : LIFT.move(0);
    //INTAKES
    (controller.get_digital(DIGITAL_R1)) ? RIGHTINTAKE.move(INTAKE_IN) && LEFTINTAKE.move(INTAKE_IN) : (controller.get_digital(DIGITAL_R2)) ? RIGHTINTAKE.move(INTAKE_OUT) && LEFTINTAKE.move(INTAKE_OUT) : RIGHTINTAKE.move(0) && LEFTINTAKE.move(0);
    //TILTER
    if(controller.get_digital(DIGITAL_UP)){ TILTER.move(TILTER_SPEED); } else if(controller.get_digital(DIGITAL_DOWN)){ if(pot.get_value() >= TILTER_MIN){ TILTER.move(-TILTER_SPEED); } } else { TILTER.move(0); }
    if(controller.get_digital(DIGITAL_X)){
      setDriveBrakes(MOTOR_BRAKE_HOLD);
      while(pot.get_value() <= ((TILTER_MAX+TILTER_MIN)/2)) { TILTER.move(TILTER_SPEED) pros::delay(10); }
      RIGHTINTAKE.move(INTAKE_IN);
      LEFTINTAKE.move(INTAKE_IN);
      motorTarget(TILTERPORT, tilterPID, 1, TILTER_MAX, 3500, 0.6, 0.02, true);
      RIGHTINTAKE.move(0);
      LEFTINTAKE.move(0);
      setDriveBrakes(MOTOR_BRAKE_COAST);
    }
    if(controller.get_digital(DIGITAL_Y)){
      motorTarget(TILTERPORT, tilterPID, 1, TILTER_MIN, 2000, 1, 0.02, false);
    }

    pros::delay(20);
  }
}
