#include "main.h"

char HYDRA_VERSION_STRING[] = "2.0.1";
int LIFT_MAX_ERROR = 10;
int TILTER_MAX_ERROR = 10;
int INTAKE_MAX_ERROR = 10;
int CHASSIS_MAX_ERROR = 10;
int CHASSIS_AUTONMOVE_HOLDTIME = 500;
int AUTONMOVE_WAIT_CHECKFREQ = 300;
double VEL_P_CHASSIS = 5.0;
double VEL_I_CHASSIS = 0.001;
double VEL_D_CHASSIS = 0.001;
double POS_P_CHASSIS = 1.0;
double POS_I_CHASSIS = 0.001;
double POS_D_CHASSIS = 0.1;
int PORT_LD_F = 11;
int PORT_RD_F = 19;
int PORT_LD_R = 12;
int PORT_RD_R = 20;
int PORT_LIFT = 3;
int PORT_TILTER = 9;
int PORT_INTAKE_L = 1;
int PORT_INTAKE_R = 10;
int PORT_TEST_MOTOR = 4;
int PORT_IMU = 13;

void load_autonomous(){
	AUTONOMOUS_SEQUENCE[0] = Autonomous_Section{move, 1080, 100, 5000};
	AUTONOMOUS_SEQUENCE[1] = Autonomous_Section{move, -1080, 100, 5000};
	AUTONOMOUS_SEQUENCE[2] = Autonomous_Section{move, 1080, 100, 5000};
	AUTONOMOUS_SEQUENCE[3] = Autonomous_Section{move, -1080, 100, 5000};
	AUTONOMOUS_SEQUENCE[4] = Autonomous_Section{end, 0, 0, 0};
	AUTONOMOUS_SEQUENCE[5] = Autonomous_Section{end};
}

void load_keyBindings(){
	KEY_MAP[0] = Key_Binding{arcade_drive, E_HYDRA_ANALOG_LEFT_Y, E_HYDRA_ANALOG_RIGHT_X};
	KEY_MAP[1] = Key_Binding{lift, E_HYDRA_DIGITAL_L1, E_HYDRA_DIGITAL_L2};
	KEY_MAP[2] = Key_Binding{tilter, E_HYDRA_DIGITAL_UP, E_HYDRA_DIGITAL_DOWN};
	KEY_MAP[3] = Key_Binding{intake, E_HYDRA_DIGITAL_R1, E_HYDRA_DIGITAL_R2};
	KEY_MAP[4] = Key_Binding{end};
}
