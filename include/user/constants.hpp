#ifndef _CONSTANTS_HPP
#define _CONSTANTS_HPP


extern char HYDRA_VERSION_STRING[];


extern double VEL_P_CHASSIS;
extern double VEL_I_CHASSIS;
extern double VEL_D_CHASSIS;
extern double POS_P_CHASSIS;
extern double POS_I_CHASSIS;
extern double POS_D_CHASSIS;


extern int LIFT_MAX_ERROR;
extern int TILTER_MAX_ERROR;
extern int INTAKE_MAX_ERROR;

extern int CHASSIS_MAX_ERROR;
extern int CHASSIS_AUTONMOVE_HOLDTIME;
extern int AUTONMOVE_WAIT_CHECKFREQ;


extern int PORT_LD_F;
extern int PORT_RD_F;
extern int PORT_LD_R;
extern int PORT_RD_R;

extern int PORT_LIFT;
extern int PORT_TILTER;
extern int PORT_INTAKE_L;
extern int PORT_INTAKE_R;

extern int PORT_TEST_MOTOR;

extern int PORT_IMU;


#endif
