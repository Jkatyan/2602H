#include "main.h"


/*struct Key_Binding* KEY_MAP;


void opcontrol_initialize(){
	KEY_MAP = (struct Key_Binding*)( lv_mem_alloc( 64 * sizeof(Autonomous_Section) ) );
	load_keyBindings();
}*/


void opcontrol() {

	using namespace okapi::literals;/*


	MOTION_PROFILER->generatePath( {{0_ft, 0_ft, 0_deg}, {1.5_ft, 1.5_ft, 90_deg}}, "A" );

	MOTION_PROFILER->setTarget("A");

	MOTION_PROFILER->waitUntilSettled();


	while(1);*/

	Shishir::Hot::CHASSIS->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	while(1){

		Shishir::Hot::chassis_drive_arcade(
			Shishir::Hot::CONTROLLER_A.getAnalog(okapi::ControllerAnalog::leftY),
			Shishir::Hot::CONTROLLER_A.getAnalog(okapi::ControllerAnalog::rightX),
		true);

		Shishir::Hot::TIMER.delay( okapi::QFrequency(30.0) );
	}
}
