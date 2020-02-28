#include "main.h"


/*struct Key_Binding* KEY_MAP;


void opcontrol_initialize(){
	KEY_MAP = (struct Key_Binding*)( lv_mem_alloc( 64 * sizeof(Autonomous_Section) ) );
	load_keyBindings();
}*/


void opcontrol() {

	using namespace okapi::literals;

	Hybot::Components::CHASSIS->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	
	Hybot::Components::MOTION_PROFILER->generatePath({ 
		{okapi::QLength(0.0), okapi::QLength(0.0), okapi::QAngle(0.0)}, 
		{okapi::QLength(0.5), okapi::QLength(0.5), okapi::QAngle(90.0)}},
	"A" );

	Hybot::Components::MOTION_PROFILER->setTarget("A", false, false);

	Hybot::Components::MOTION_PROFILER->waitUntilSettled();

	Hybot::Components::TIMER.delayUntil(1000);

	Hybot::Components::MOTION_PROFILER->setTarget("A", true, true);

	Hybot::Components::MOTION_PROFILER->waitUntilSettled();

	Hybot::Components::TIMER.delayUntil(1000);



	/*Hybot::Components::CHASSIS->driveVector(0.2, 0.05);
	Hybot::Components::TIMER.delayUntil(3000);

	Hybot::Components::CHASSIS->stop();
	Hybot::Components::TIMER.delayUntil(250);

	Hybot::Components::CHASSIS->driveVector(-0.2, -0.05);
	Hybot::Components::TIMER.delayUntil(3000);

	Hybot::Components::CHASSIS->stop();

	while(1);*/

	while(1){

		Hybot::API::chassis_drive_arcade(
			Hybot::Components::CONTROLLER_A.getAnalog(okapi::ControllerAnalog::leftY),
			Hybot::Components::CONTROLLER_A.getAnalog(okapi::ControllerAnalog::rightX)
		);

		Hybot::Components::TIMER.delay( okapi::QFrequency(30.0) );
	}
}
