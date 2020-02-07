#include "main.h"

char HYDRA_VERSION_STRING[] = "2.0.1";
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
int LIFT_MAX_ERROR = 10;
int TILTER_MAX_ERROR = 10;
int INTAKE_MAX_ERROR = 10;
int CHASSIS_MAX_ERROR = 10;
int CHASSIS_AUTONMOVE_HOLDTIME = 200;
int AUTONMOVE_WAIT_CHECKFREQ = 300;

void load_autonomous(){
	AUTONOMOUS_SEQUENCE[0] = Autonomous_Section{move, 1080, 200, 5000};
	AUTONOMOUS_SEQUENCE[1] = Autonomous_Section{turn, 180, 200, 5000};
	AUTONOMOUS_SEQUENCE[2] = Autonomous_Section{move, 360, 200, 5000};
	AUTONOMOUS_SEQUENCE[3] = Autonomous_Section{move, -360, 200, 5000};
	AUTONOMOUS_SEQUENCE[4] = Autonomous_Section{turn, -180, 200, 5000};
	AUTONOMOUS_SEQUENCE[5] = Autonomous_Section{move, -1080, 200, 5000};
	AUTONOMOUS_SEQUENCE[6] = Autonomous_Section{end, 0, 0, 0};
	AUTONOMOUS_SEQUENCE[7] = Autonomous_Section{end};
}
