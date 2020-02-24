#include "main.h"


struct Key_Binding* KEY_MAP;


void opcontrol_initialize(){
	KEY_MAP = (struct Key_Binding*)( lv_mem_alloc( 64 * sizeof(Autonomous_Section) ) );
	load_keyBindings();
}


void opcontrol() {
	using namespace okapi::literals;

	okapi::MotorGroup MG({(const okapi::Motor&)LD_F, (const okapi::Motor&)LD_R});

	/*std::shared_ptr<okapi::ChassisController> myChassis = okapi::ChassisControllerBuilder()//:
		.withMotors({-1, -2}, {3, 4})
		.withDimensions( okapi::AbstractMotor::gearset::green, {{4_in, 11.5_in}, okapi::imev5GreenTPR} )
		.build();*/

	while(1){
		/*pass*/;
	}
}
