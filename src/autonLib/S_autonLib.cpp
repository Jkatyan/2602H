#include "autons.hpp"

bool S_motorSuccess( pros::Motor motor ){
	int diff = (int) motor.get_position() - motor.get_target_position();

	if( diff > (-1) * AUTOMOVE_ALLOWABLE_ERROR && diff < AUTOMOVE_ALLOWABLE_ERROR ){
		return true;
	}
	else{
		return false;
	}
}


int S_chassis_wait_till_success(int timeOut, int mode){
	uint32_t startTime = pros::millis();
	int successCount = 0;
	int display_update_count = 0;

	while(1){
		if( successCount > AUTOMOVE_SUCCESS_HOLDING_TIME ){
			return 0;
		}
		if( pros::millis() - startTime > timeOut ){
			return -1;
		}
		if( S_motorSuccess(LD) && S_motorSuccess(LD2) && S_motorSuccess(RD) && S_motorSuccess(RD2) ){
			successCount++;
		}
		else{
			successCount = 0;
		}

		if( display_update_count <= ( 1000 / LCD_DISPLAY_FRAMERATE ) ){
			display_update_count++;
		}
		else{
			if(mode == 0){
				pros::lcd::print( 0, "command: move" );
			}
			else{
				pros::lcd::print( 0, "command: turn" );
			}
			pros::lcd::print(1, "target: %f", LD.get_target_position() );
			pros::lcd::print(2, "actual: %f", LD.get_position() );
			pros::lcd::print(3, "success: %d", successCount > 0 );
			pros::lcd::print(4, "successHold: %d", AUTOMOVE_SUCCESS_HOLDING_TIME - successCount );
			pros::lcd::print(5, "timeOut: %d", startTime + timeOut - pros::millis() );
			display_update_count = 0;
		}

		pros::delay(1);
	}
}


int S_chassis_move(int angle, float speed, int timeOut){
	int s = (int) (speed * 127);

	LD.move_relative(angle, s);
	RD.move_relative(angle, s);
	LD2.move_relative(angle, s);
	RD2.move_relative(angle, s);

	return S_chassis_wait_till_success(timeOut, 0);
}


int S_chassis_turn(int angle, float speed, int timeOut){
	int s = (int) (speed * 127);

	LD.move_relative(angle, s);
	RD.move_relative((-1)*angle, s);
	LD2.move_relative(angle, s);
	RD2.move_relative((-1)*angle, s);

	return S_chassis_wait_till_success(timeOut, 1);
}


void S_zero_all_motors(){
	LD.move(0);
	RD.move(0);
	LD2.move(0);
	RD2.move(0);
	TILTER.move(0);
	LIFT.move(0);
	INTAKEA.move(0);
	INTAKEB.move(0);
}

void S_reset_all_motors(){
	LD.tare_position();
	LD2.tare_position();
	RD.tare_position();
	RD2.tare_position();
	LIFT.tare_position();
	INTAKEA.tare_position();
	INTAKEB.tare_position();
	TILTER.tare_position();
}

void S_reset_chassis_motors(){
	LD.tare_position();
	LD2.tare_position();
	RD.tare_position();
	RD2.tare_position();
}

void S_drive_chassis_tank(){
	LD.move(LC*(controller.get_analog(ANALOG_LEFT_Y)));
	LD2.move(LC*(controller.get_analog(ANALOG_LEFT_Y)));
	RD.move(RC*(controller.get_analog(ANALOG_RIGHT_Y)));
	RD2.move(RC*(controller.get_analog(ANALOG_RIGHT_Y)));
}


void S_drive_chassis_arcade(){
	int x = controller.get_analog(ANALOG_RIGHT_Y);
	int y = controller.get_analog(ANALOG_RIGHT_X);
	LD.move(x+y);
	RD.move(x-y);
	LD2.move(x+y);
	RD2.move(x-y);
}


void S_moveMotor_withLimit(pros::Motor motor, int velocity, int max_value, int min_value,
		pros::controller_digital_e_t button1, pros::controller_digital_e_t button2, int limitSource){
	int armPoz;
	if( limitSource == 1 ){
		armPoz = motor.get_position();
	}
	else if( limitSource == 2 ){
		armPoz = pot.get_value();
	}

	if( controller.get_digital(button1) && ( ( armPoz < max_value ) || (!limitSource) ) ){
		motor.move(velocity);
	}
	else if( controller.get_digital(button2) && ( ( armPoz > min_value ) || (!limitSource) ) ){
		motor.move((-1)*velocity);
	}
	else{
		motor.move_velocity(0);
	}
	//pros::lcd::print(0, "motorPoz: %d", armPoz);
}


void S_armsMotion_proceed(){
	//S_moveMotor_withLimit(TILTER, TILTER_SPEED, TILTER_MAX_VALUE, TILTER_MIN_VALUE, DIGITAL_L1, DIGITAL_L2, 1);
	//S_moveMotor_withLimit(LIFT, LIFT_SPEED, LIFT_MAX_VALUE, LIFT_MIN_VALUE, DIGITAL_L1, DIGITAL_L2, 1);
	S_moveMotor_withLimit(INTAKEA, INTAKEA_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
	S_moveMotor_withLimit(INTAKEB, INTAKEB_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
}


void S_armsMotion_Amode_proceed(){
	TILTER.move( (-1) * controller.get_analog(ANALOG_LEFT_Y) );
	S_moveMotor_withLimit(LIFT, LIFT_SPEED, LIFT_MAX_VALUE, LIFT_MIN_VALUE, DIGITAL_UP, DIGITAL_DOWN, 1);
	S_moveMotor_withLimit(INTAKEA, INTAKEA_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
	S_moveMotor_withLimit(INTAKEB, INTAKEB_SPEED, 0, 0, DIGITAL_R1, DIGITAL_R2, 0);
}
