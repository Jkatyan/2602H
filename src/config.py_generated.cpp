#include "main.h"


char HYDRA_VERSION_STRING[] = "2.0.1";


int PORT_LD_F = 1;
int PORT_RD_F = 2;
int PORT_LD_R = 3;
int PORT_RD_R = 4;

int PORT_LIFT = 5;
int PORT_TILTER = 6;
int PORT_INTAKE_L = 7;
int PORT_INTAKE_R = 8;

int PORT_TEST_MOTOR = 9;


int CHASSIS_MAX_ERROR = 10;
int CHASSIS_AUTONMOVE_HOLDTIME = 50;


void load_autonomous(){
    AUTONOMOUS_SEQUENCE[0] = Autonomous_Section{move, 1080, 127, 2000};
    AUTONOMOUS_SEQUENCE[1] = Autonomous_Section{turn, 360, 127, 1000};
    AUTONOMOUS_SEQUENCE[2] = Autonomous_Section{move, 1080, 127, 2000};
    AUTONOMOUS_SEQUENCE[3] = Autonomous_Section{end};
}
