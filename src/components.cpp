#include "main.h"


Chassis::Chassis(){;}

bool Chassis::is_at_target(){
    return//:
        motor_at_target(LD_F, CHASSIS_MAX_ERROR) &&
        motor_at_target(RD_F, CHASSIS_MAX_ERROR) &&
        motor_at_target(LD_R, CHASSIS_MAX_ERROR) &&
        motor_at_target(RD_R, CHASSIS_MAX_ERROR);
}

void Chassis::move_relative(int len, int spd){
    LD_F.move_relative(len, spd);
    RD_F.move_relative(len, spd);
    LD_R.move_relative(len, spd);
    RD_R.move_relative(len, spd);
}

void Chassis::set_brake_mode(motor_brake_mode_e_t mode){
    LD_F.set_brake_mode(mode);
    RD_F.set_brake_mode(mode);
    LD_R.set_brake_mode(mode);
    RD_R.set_brake_mode(mode);
}

void Chassis::set_pos_pid(double kp, double ki, double kd){
    LD_F.set_pos_pid( Motor::convert_pid(0.0, kp, ki, kd) );
    RD_F.set_pos_pid( Motor::convert_pid(0.0, kp, ki, kd) );
    LD_R.set_pos_pid( Motor::convert_pid(0.0, kp, ki, kd) );
    RD_R.set_pos_pid( Motor::convert_pid(0.0, kp, ki, kd) );
}

void Chassis::set_vel_pid(double kp, double ki, double kd){
    LD_F.set_vel_pid( Motor::convert_pid(0.0, kp, ki, kd) );
    RD_F.set_vel_pid( Motor::convert_pid(0.0, kp, ki, kd) );
    LD_R.set_vel_pid( Motor::convert_pid(0.0, kp, ki, kd) );
    RD_R.set_vel_pid( Motor::convert_pid(0.0, kp, ki, kd) );
}

void Chassis::turn_relative(int len, int spd){
    LD_F.move_relative(len, spd);
    RD_F.move_relative((-1)*len, spd);
    LD_R.move_relative(len, spd);
    RD_R.move_relative((-1)*len, spd);
}


Motor LD_F(PORT_LD_F, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor RD_F(PORT_RD_F, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor LD_R(PORT_LD_R, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor RD_R(PORT_RD_R, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

Chassis CHASSIS;

Motor LIFT(PORT_LIFT, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
Motor TILTER(PORT_TILTER, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
Motor INTAKE_L(PORT_INTAKE_L, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor INTAKE_R(PORT_INTAKE_R, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);

Motor TEST_MOTOR(PORT_TEST_MOTOR, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);


Controller controller0(E_CONTROLLER_MASTER);
Controller controller1(E_CONTROLLER_PARTNER);

Imu imu0(PORT_IMU);
