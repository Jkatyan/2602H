#include "main.h"
#include "user/components.hpp"


Motor testMotor(7, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);


Controller controller0(E_CONTROLLER_MASTER);
Controller controller1(E_CONTROLLER_PARTNER);
