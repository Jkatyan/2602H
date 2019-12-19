#include "main.h"


pros::ADIGyro gyro (GYROPORT, GC);
pros::ADIAnalogIn pot (POTPORT);

pros::ADIEncoder sideEnc('G', 'H', true);
pros::ADIEncoder leftEnc('F', 'E');
pros::ADIEncoder rightEnc('A', 'B', true);

pros::ADIEncoder sideEncOdom('G', 'H', true);
pros::ADIEncoder leftEncOdom('F', 'E');
pros::ADIEncoder rightEncOdom('A', 'B', true);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Controller second_controller(pros::E_CONTROLLER_PARTNER);
pros::Motor LD(LDPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor LD2(LD2PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RD(RDPORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor RD2(RD2PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor INTAKEA(INTAKEAPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor INTAKEB(INTAKEBPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor TILTER(TILTERPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor LIFT(LIFTPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
