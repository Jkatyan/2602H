#ifndef ODOMETRY_CPP
#define ODOMETRY_CPP

#include "main.h"

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
  curLeft = leftEncOdom.get_value();curRight = rightEncOdom.get_value();curSide = sideEncOdom.get_value();
  deltaLeft = (curLeft - lastLeftPos)*(M_PI/180)*(wheelDiameter/2);deltaRight = (curRight - lastRightPos)*(M_PI/180)*(wheelDiameter/2);deltaSide = (curSide - lastSidePos)*(M_PI/180)*(trackingDiameter/2);
  lastLeftPos = curLeft;lastRightPos = curRight; lastSidePos = curSide;
  deltaLr = (curLeft - leftAtReset)*(M_PI/180)*(wheelDiameter/2);deltaRr = (curRight - rightAtReset)*(M_PI/180)*(wheelDiameter/2);
  thetaNew = (thetaReset + (deltaLr - deltaRr)/(Sl + Sr));
  deltaTheta = thetaNew - angle;
  deltaSide = deltaSide-Ss*deltaTheta;
  if (deltaTheta == 0) {
    deltaX = deltaSide;
    deltaY = deltaRight;
  }
  else {
    deltaX = (2*sin(deltaTheta/2))*(deltaSide/deltaTheta + Ss);
    deltaY = (2*sin(deltaTheta/2))*(deltaRight/deltaTheta +Sr);
  }
  thetaM = angle + deltaTheta/2;
  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  theta = theta-thetaM;
  deltaX = radius*cos(theta);deltaY = radius*sin(theta);
  thetaNew+=M_PI;
  while (thetaNew <= 0) {
    thetaNew+=2*M_PI;
  }
  thetaNew = modulo(thetaNew, 2*M_PI);
  thetaNew-=M_PI;
  angle = thetaNew;
  x = x - deltaX;
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
