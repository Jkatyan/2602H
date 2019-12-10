#include "autons.hpp"

PID drivePID;
PID turnPID;

float lastSlewTime;
float maxAccel = 0.14;
float lastSlewRate = 0;
