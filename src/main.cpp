#include "main.h"

/*2602H Jscript*/
//Jscript Branch 2602H Github

//PORTS
#define RFPORT 20
#define RBPORT -19
#define LFPORT 11
#define LBPORT -12

#define TILTERPORT 5
#define LIFTPORT 6
#define RINTPORT 7
#define LINTPORT -8

#define IMUPORT 15

//INITIALIZE
pros::Imu imu(IMUPORT);

Motor rf(RFPORT);
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
ControllerButton intakeIn(ControllerDigital::R1);
ControllerButton intakeOut(ControllerDigital::R2);
ControllerButton liftUp(ControllerDigital::L1);
ControllerButton liftDown(ControllerDigital::L2);

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
        {{0_ft, 0_ft, 0_deg}, {1_ft, 0_ft, 0_deg}}, "A");
      profileController->generatePath(
        {{1_ft, 0_ft, 0_deg}, {3_ft, -0.7_ft, 0_deg}}, "B");
    }

//DRIVE FUNCTIONS
void setDrive(int left, int right){
rf.moveVoltage(right);
rb.moveVoltage(right);
lf.moveVoltage(left);
lb.moveVoltage(left);
}

void rotate(int degrees, int voltage){
  if (imu.get_rotation() < degrees){
    while (imu.get_rotation() < degrees){
      setDrive(voltage, -voltage);
      pros::delay(10);
    }
    setDrive(0,0);
    pros::delay(100);
    while (imu.get_rotation() > degrees){
      setDrive(0.5 * voltage, -0.5 * voltage);
      pros::delay(10);
    }
    setDrive(0,0);
}
else {
    while (imu.get_rotation() > degrees){
      setDrive(-voltage, voltage);
      pros::delay(10);
    }
    setDrive(0,0);
    pros::delay(100);
    while (imu.get_rotation() < degrees){
      setDrive(-0.5 * voltage, 0.5 * voltage);
      pros::delay(10);
    }
    setDrive(0,0);
  }
}

//MAIN (AUTON) CODE
void autonomous(){
  //rotate(10,3000);
  profileController->setTarget("A"); //Set true to follow backwards
  profileController->waitUntilSettled();
  //profileController->setTarget("A", true, true); //Set true to follow backwards
  //profileController->waitUntilSettled();
  profileController->removePath("A");
  profileController->setTarget("B"); //Set true to follow backwards
  profileController->waitUntilSettled();
  profileController->removePath("B");
}
void opcontrol() {
  lift.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(AbstractMotor::brakeMode::brake);
  while(true){
    myChassis->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));
  }
}
