#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "main.h"

extern void updatePosition();
extern float getX();
extern float getY();
extern float getAngle();
extern float getAngleDegrees();
extern float modulo(float a, float b);

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

#endif
