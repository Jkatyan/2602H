#include "autons.hpp"

PID drivePID;
PID turnPID;
PID tilterPID;
PID liftPID;

float lastSlewTime;
float maxAccel = 0.17; //Chassis
float maxAccelMotor = 0;
float lastSlewRate = 0;

pros::Motor aLD(LDPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor aLD2(LD2PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor aRD(RDPORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor aRD2(RD2PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor aINTAKEA(INTAKEAPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor aINTAKEB(INTAKEBPORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor aTILTER(TILTERPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor aLIFT(LIFTPORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

void A_motorTarget(int port, PID pid, int special, int target, int time, float speed, float accel, bool slew){
  pros::Motor motor(port);
  int atTarget = 0;
  int encoder = 0;
  int startTime = pros::millis();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
  switch(special){case 0:encoder = motor.get_position();break;case 1:encoder = pot.get_value();break;default:break;}
  float val = pidCalculate(pid, target, encoder)*speed;
  val = (slew)? motorSlew(val, accel): val;
  motor.move(val);
  if(encoder == target){
    atTarget = 1;
  }
  pros::delay(15);
  }
  motor.move(0);
}

float motorSlew (float desiredRate, float maxAccelMotor) {
    		float deltaTime = pros::millis()-lastSlewTime;
    		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
    		float addedRate;
    		float newRate;

    		if (fabs(desiredAccel) < maxAccelMotor || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
    		    addedRate = desiredAccel*deltaTime;
    		    newRate = addedRate+lastSlewRate;
    		}
    		else {
    		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccelMotor*deltaTime;
            newRate = addedRate+lastSlewRate;
    		}
    	  lastSlewTime = lastSlewTime+deltaTime;
    	  lastSlewRate = newRate;

    		float returnVal = newRate;
    		return returnVal;}

void A_gyroDriveTarget(float angle, int target, int time, float speed){
  int atTarget = 0;
  int driveEnc = 0;
  int startTime = pros::millis();

  while ((atTarget != 1) && (pros::millis()-startTime) < time) {
  driveEnc = ((abs((aLD2.get_position() + aLD.get_position())/2))+(abs((aRD2.get_position() + aRD.get_position()))/2))/2;
  // SETTINGdriveEnc = (abs(LD.get_position())+abs(RD.get_position()))/2;
      float val = pidCalculate(drivePID, target, -driveEnc)*speed;
      val = slewRateCalculate(val);
      int rightVal = val - pidCalculate(turnPID, angle, gyro.get_value()/10.0);
      int leftVal = val + pidCalculate(turnPID, angle, gyro.get_value()/10.0);
      /*pros::lcd::print(3, "Gyro Angle: %f", (gyro.get_value()/10.0000));
  		pros::lcd::print(4, "driveEnc: %d", driveEnc);*/

        aRD.move(RC*rightVal);
        aRD2.move(RC*rightVal);
        aLD.move(LC*leftVal);
        aLD2.move(LC*leftVal);
        if(driveEnc == target){
          atTarget = 1;
        }
      pros::delay(15);
    } //While Loop

    aLD.set_zero_position(driveEnc-target); aLD.move(0);
    aLD2.set_zero_position(driveEnc-target); aLD2.move(0);
    aRD.set_zero_position(driveEnc-target); aRD.move(0);
    aRD2.set_zero_position(driveEnc-target); aRD2.move(0);
}

void A_gyroTurn(int target, int accuracy, int time, float speed) {
  gyro.reset();
	int startTime = pros::millis();
	bool gyroAtTarget = false;
	int repsAtTarget = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!gyroAtTarget  && (pros::millis()-startTime) < time) {
		// calculate the desired motor value based on the sensor value relative to the target
		float drive = pidCalculate(turnPID, target, gyro.get_value()/10.0)*speed;
		drive = ((fabs(gyro.get_value()/10.0-target)>180)? -1 : 1)*drive;
    aLD.move(drive);
    aLD2.move(drive);
    aRD.move(-drive);
    aRD2.move(-drive);
		//if the sensor value is within the desired range of the target
		if (fabs(gyro.get_value()/10.0-target) < accuracy) {
			//if the sensor value is within the range for multiple iterations of the loop where each loop is approximately 20ms
			if (repsAtTarget > 15) {
				//break out of the loop
				gyroAtTarget = true;
			}
			else {
				repsAtTarget++;
			}
		}
		else {
			repsAtTarget = 0;
		}
		pros::delay(15);
	}
	S_zero_all_motors();
}

void A_goTo(float targetX, float targetY) {
	bool atPoint = false;
	float targetAngle =0;
	float power =0;
	float turnPower =0;
	lastSlewTime = pros::millis();

	while (!atPoint) {
		updatePosition();

		power = -pidCalculate(drivePID, 0, sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)));
		power = slewRateCalculate(power);

		targetAngle = atan2f((targetY-getY()),(targetX-getX()))-M_PI/2;
		if (targetAngle >= M_PI) {
	    targetAngle-=2*M_PI;
	  }
	  else if (targetAngle <= -M_PI) {
	    targetAngle+=2*M_PI;
	  }

		turnPower = ((fabs(targetAngle-getAngle())>M_PI)? -1: 1)*pidCalculate(turnPID, targetAngle, getAngle());

		aLD.move((power + turnPower)*LC);
    aRD.move((power - turnPower)*RC);
		aLD2.move((power + turnPower)*LC);
		aRD2.move((power - turnPower)*RC);

		pros::lcd::print(0, "X: %f", getX());
		pros::lcd::print(1, "Y: %f", getY());
		pros::lcd::print(2, "Angle: %f", getAngleDegrees());

		pros::lcd::print(7, "Target Angle: %f", targetAngle*180/M_PI);


		if (sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)) < 3 || controller.get_digital(DIGITAL_B)) {
			atPoint = true;
		}
		pros::delay(10);
	}
}

float slewRateCalculate (float desiredRate) {
		//pros::lcd::print(7, "called: %f", desiredRate);
		float deltaTime = pros::millis()-lastSlewTime;
		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
		float addedRate;
		float newRate;

		if (fabs(desiredAccel) < maxAccel || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
		    addedRate = desiredAccel*deltaTime;
		    newRate = addedRate+lastSlewRate;
		}
		else {
		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccel*deltaTime;
        newRate = addedRate+lastSlewRate;
		}
	  lastSlewTime = lastSlewTime+deltaTime;
	  lastSlewRate = newRate;

		float returnVal = newRate;
		return returnVal;
}

void setDrive(int left, int right){
	aLD = left;
	aLD2 = left;
	aRD = right;
	aRD2 = right;
}

void A_rotate(int degrees, int voltage){
	int direction = abs(degrees) / degrees;
	gyro.reset();
	setDrive(-voltage * direction, voltage * direction);
	while(fabs(gyro.get_value()) < abs(degrees * 10) - 50){
		pros::delay(10);
	}
	pros::delay(100);
	if(fabs(gyro.get_value()) > abs(degrees * 10)){
		setDrive(0.5 * voltage * direction, 0.5 * -voltage * direction);
		while(fabs(gyro.get_value()) > abs(degrees * 10)){
			pros::delay(10);
		}
	}
	else if(fabs(gyro.get_value()) < abs(degrees * 10)){
		setDrive(0.5 * -voltage * direction, 0.5 * voltage * direction);
		while(fabs(gyro.get_value()) > abs(degrees * 10)){
			pros::delay(10);
		}
	}
	setDrive(0,0);
}

void A_driveTarget(int target, int time, float speed){
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();

    while ((atTarget != 1) && (pros::millis()-startTime) < time) {
    driveEnc = ((abs(aLD.get_position()))+(abs(aRD.get_position())))/2;
    distance = target - driveEnc;

		pros::lcd::print(1, "Drive Encoder Value: %f", driveEnc);
		pros::lcd::print(2, "Distance from Target: %f", distance);
    pros::lcd::print(4, "Right Encoder: %d", aRD.get_position());
		pros::lcd::print(5, "Left Encoder: %d", aLD.get_position());

    float val = pidCalculate(drivePID, target, driveEnc)*speed;
    val = slewRateCalculate(val);
    int rightVal = val;
    int leftVal = val;

    aRD.move(RC*rightVal);
    aRD2.move(RC*rightVal);
    aLD.move(LC*leftVal);
    aLD2.move(LC*leftVal);
    if(driveEnc == target){
       atTarget = 1;
       pros::lcd::print(2, "Distance from Target: %f", distance);
      }
      pros::delay(20);
    }
}
