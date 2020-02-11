#include "main.h"

auto myChassis =
  ChassisControllerBuilder()
    .withMotors({1, 2}, {-3, -4})
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


void opcontrol() {
  profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {0_ft, 0_ft, 0_deg}}, "A");
  profileController->setTarget("A"); //Set true to follow backwards
  profileController->waitUntilSettled();
  profileController->removePath("A");
}
