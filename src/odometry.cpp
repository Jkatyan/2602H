#ifndef ODOMETRY_CPP
#define ODOMETRY_CPP

#include "main.h"

pros::ADIEncoder sideEncOdom('G', 'H', true);
pros::ADIEncoder leftEncOdom('F', 'E');
pros::ADIEncoder rightEncOdom('A', 'B', true);

/* PILONS Code:
void trackPosition(int left, int right, int back, sPos& position)
{
	float L = (left - position.leftLst) * SPIN_TO_IN_LR; // The amount the left side of the robot moved
	float R = (right - position.rightLst) * SPIN_TO_IN_LR; // The amount the right side of the robot moved
	float S = (back - position.backLst) * SPIN_TO_IN_S; // The amount the back side of the robot moved

	// Update the last values
	position.leftLst = left;
	position.rightLst = right;
	position.backLst = back;

	float h; // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
	float i; // Half on the angle that I've traveled
	float h2; // The same as h but using the back instead of the side wheels
	float a = (L - R) / (L_DISTANCE_IN + R_DISTANCE_IN); // The angle that I've traveled
	if (a)
	{
		float r = R / a; // The radius of the circle the robot travel's around with the right side of the robot
		i = a / 2.0;
		float sinI = sin(i);
		h = ((r + R_DISTANCE_IN) * sinI) * 2.0;

		float r2 = S / a; // The radius of the circle the robot travel's around with the back of the robot
		h2 = ((r2 + S_DISTANCE_IN) * sinI) * 2.0;
	}
	else
	{
		h = R;
		i = 0;

		h2 = S;
	}
	float p = i + position.a; // The global ending angle of the robot
	float cosP = cos(p);
	float sinP = sin(p);

	// Update the global position
	position.y += h * cosP;
	position.x += h * sinP;

	position.y += h2 * -sinP; // -sin(x) = sin(-x)
	position.x += h2 * cosP; // cos(x) = cos(-x)

	position.a += a;
}
*/

float Sl = 6.625; //distance from tracking center to middle of left wheel
float Sr = 6.594; //distance from tracking center to middle of right wheel
float Ss = 4.344; //distance from tracking center to middle of the tracking wheel
float wheelDiameter = 3.25; //diameter of the side wheels being used for tracking
float trackingDiameter = 3.25; //diameter of the sideways tracking wheel

float x = 0;
float y = 0;
float angle = 0;;

float lastLeftPos = 0;
float lastRightPos = 0;
float lastSidePos = 0;

float deltaTheta = 0;
float thetaNew = 0;
float thetaM = 0;

float curLeft = 0;
float curRight = 0;
float curSide = 0;

float leftAtReset = 0;
float rightAtReset = 0;
float thetaReset = 0;

float deltaLeft = 0;
float deltaRight = 0;
float deltaSide = 0;

float deltaLr = 0;
float deltaRr = 0;

float deltaX = 0;
float deltaY = 0;

float theta = 0;
float radius = 0;

void updatePosition() {
  curLeft = leftEncOdom.get_value();
  curRight = rightEncOdom.get_value(); //step 1
  curSide = sideEncOdom.get_value();

  deltaLeft = (curLeft - lastLeftPos)*(M_PI/180)*(wheelDiameter/2);
  deltaRight = (curRight - lastRightPos)*(M_PI/180)*(wheelDiameter/2); //step 2
  deltaSide = (curSide - lastSidePos)*(M_PI/180)*(trackingDiameter/2);

  lastLeftPos = curLeft;
  lastRightPos = curRight; //step 3
  lastSidePos = curSide;

  deltaLr = (curLeft - leftAtReset)*(M_PI/180)*(wheelDiameter/2); //step 4
  deltaRr = (curRight - rightAtReset)*(M_PI/180)*(wheelDiameter/2);

  thetaNew = (thetaReset + (deltaLr - deltaRr)/(Sl + Sr)); //step 5
  // if (thetaNew >= M_PI) {
  //   thetaNew-=2*M_PI;
  // }
  // else if (thetaNew <= -M_PI) {
  //   thetaNew+=2*M_PI;
  // }



  deltaTheta = thetaNew - angle; //step 6

  deltaSide = deltaSide-Ss*deltaTheta;

  if (deltaTheta == 0) {
    deltaX = deltaSide; //step 7
    deltaY = deltaRight;
  }
  else {
    deltaX = (2*sin(deltaTheta/2))*(deltaSide/deltaTheta + Ss); //step 8
    deltaY = (2*sin(deltaTheta/2))*(deltaRight/deltaTheta +Sr);
  }

  thetaM = angle + deltaTheta/2; //step 9

  // if (deltaX == 0) {
  //   if (deltaY > 0) {
  //     theta = M_PI/2;
  //   }
  //   else if (deltaY < 0) {
  //     theta = 3*M_PI/2;
  //   }
  //   else {
  //     theta = 0;
  //   }
  // }
  // else {
  //   theta = atan(deltaY/deltaX);
  // }
  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  theta = theta-thetaM;                          //step 10
  deltaX = radius*cos(theta);
  deltaY = radius*sin(theta);

  thetaNew+=M_PI;
  while (thetaNew <= 0) {
    thetaNew+=2*M_PI;
  }
  thetaNew = modulo(thetaNew, 2*M_PI);
  thetaNew-=M_PI;

  angle = thetaNew;
  x = x - deltaX; //step 11
  y = y + deltaY;
}

float getX() {
  return x;
}

float getY() {
  return y;
}

float getAngleDegrees() {
  return angle*180/M_PI;
}
float getAngle() {
  return angle;
}

float modulo(float a, float b) {
  while (a>b) {
    a-=b;
  }
  return a;
}

#endif
