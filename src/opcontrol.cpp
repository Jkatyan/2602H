#include "main.h"

/*2602H Jscript*/

//PORTS
#define RFPORT 1
#define RBPORT 2
#define LFPORT 3
#define LBPORT 4

#define IMUPORT 1

//INITIALIZE
pros::Imu imu(IMUPORT);

pros::Motor rf(RFPORT);
pros::Motor rb(RBPORT, true);
pros::Motor lf(LFPORT, true);
pros::Motor lb(LBPORT);

void initialize() {
  imu.reset();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "2602H JScript");
}

void disabled() {}
void competition_initialize() {}

//MOTION PROFILE INITIALIZE
auto myChassis =
  ChassisControllerBuilder()
    .withMotors({RFPORT, -RBPORT}, {-LFPORT, LBPORT})
    // Green Gearset, 3.25 in wheel diam, 10 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10_in}, imev5GreenTPR})
    .build();

auto profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      0.5,  // Maximum linear velocity of the Chassis in m/s
      1.5,  // Maximum linear acceleration of the Chassis in m/s/s
      8.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(myChassis)
    .buildMotionProfileController();

//DRIVE FUNCTIONS
void setDrive(int left, int right){
rf = right;
rb = right;
lf = left;
lb = left;
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
void opcontrol() {
  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 0_deg}}, "A");
  profileController->setTarget("A"); //Set true to follow backwards
  profileController->waitUntilSettled();
  profileController->removePath("A");
  rotate(0, 40);
}
