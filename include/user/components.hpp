#ifndef _COMPONENTS_HPP
#define _COMPONENTS_HPP

#include "okapi/api.hpp"


namespace Shishir{
    namespace Hot{
        extern okapi::Controller CONTROLLER_A;
        extern okapi::Controller CONTROLLER_B;


        extern std::shared_ptr<okapi::ChassisModel> CHASSIS;

        extern okapi::Motor LIFT;
        extern okapi::Motor TILTER;

        extern okapi::MotorGroup INTAKE;


        extern std::shared_ptr<okapi::AsyncMotionProfileController> MOTION_PROFILER;


        extern okapi::Rate TIMER;
    }
}


#endif
