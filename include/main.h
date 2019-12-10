/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2018, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
#include "okapi/api.hpp"


//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif

extern pros::Controller controller;
extern pros::Motor LD;
extern pros::Motor RD;
extern pros::Motor LD2;
extern pros::Motor RD2;
extern pros::Motor INTAKEA;
extern pros::Motor INTAKEB;
extern pros::Motor LIFT;
extern pros::Motor TILTER;

extern pros::ADIAnalogIn pot;
extern pros::ADIGyro gyro;

extern pros::ADIEncoder sideEnc;
extern pros::ADIEncoder leftEnc;
extern pros::ADIEncoder rightEnc;

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

const bool DISABLE_AUTONOMOUS = false;


void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
 #include "odometry.hpp"
 #include "PID.hpp"
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
