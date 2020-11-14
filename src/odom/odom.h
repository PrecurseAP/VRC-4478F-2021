#ifndef __ODOM__
#define __ODOM__
#include <iostream> //yo my slime, i know you dont really know mandem like that, but i was wondering if, like, i could purchase summin styl fam. just a bit of grub my drilla
#include "vex.h"
#include "../src/aidenmath/custommath.h"
#include <vex_timer.h>
#include "../src/drive/drive.h"

timer distanceCheck;
timer turnCheck;

struct robotPosition {
  float x;
  float y;
  float theta;
} pos;
float averageOrientation;
float rX = 0;
float rY = 0;

float prevTheta = 0; //variables for the gyro angle
void resetPos() {
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
  rX = 0;
  rY = 0;
  prevTheta = 0;
  averageOrientation = 0;
}



void updatePositionVars() {
  pos.x = rX;
  pos.y = rY;
  pos.theta = prevTheta * radToDeg;
}

const float wheelCirc = 2.75 * M_PI; //circumference of a 2.75" omni wheel, the ones we use for tracking wheels. This is used to calculate the distance a wheel has traveled.

const float centerToLeft = 6.0; 
const float centerToRight = 6.0; //all units in inches
const float centerToBack = 4.75;

double deltaLeft, deltaRight, deltaBack;

float leftEnc, rightEnc, backEnc;
float prevLeft = 0, prevRight = 0, prevBack = 0;

float localOffset[2]; //arrays to substitute as column vectors
float globalOffset[2];

int tracking() {
  leftEncoder.setPosition(0, degrees);
  rightEncoder.setPosition(0, degrees);
  backEncoder.setPosition(0, degrees);

  while(true) {
    leftEnc = -leftEncoder.position(degrees);
    rightEnc = rightEncoder.position(degrees); //store encoder values
    backEnc = -backEncoder.position(degrees);

    deltaLeft = ((leftEnc - prevLeft)/360) * wheelCirc;
    deltaRight = ((rightEnc - prevRight)/360) * wheelCirc; //calculate distance traveled by each tracking wheel represented in inches
    deltaBack = ((backEnc - prevBack)/360) * wheelCirc;

    prevLeft = leftEnc;
    prevRight = rightEnc; //store previous values
    prevBack = backEnc;

    float hyp, hyp2; //declare variables for the change in distance in each direction (i think)

    float deltaTheta = (deltaLeft - deltaRight) / 12.0; //change in orientation

    if (deltaTheta) {
      float sinDT = sin(deltaTheta / 2.0);
      hyp = (((deltaRight / deltaTheta) + centerToRight) * sinDT) * 2.0;  //calculate new local offset if the robot performs a turning motion
      hyp2 = (((deltaBack / deltaTheta) + centerToBack) * sinDT) * 2.0;
    } else {
      hyp = deltaRight;
      hyp2 = deltaBack; //if the robot doesnt turn, no calculation is needed
    }

    float averageOrientation = (deltaTheta / 2.0) + prevTheta; //calculate the average orientation
    
    float cosAO = cos(averageOrientation);
    float sinAO = sin(averageOrientation); 

    rY += hyp * cosAO;                        //calculate the final change in position by rotating the local offset by the average orientation
    rX += hyp * sinAO;

    rY += hyp2 * -sinAO;
    rX += hyp2 * cosAO;

    prevTheta += deltaTheta;
    
    /*Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(rX);
    Brain.Screen.newLine();
    Brain.Screen.print(rY);
    Brain.Screen.newLine();
    //Brain.Screen.print(theta);
    Brain.Screen.newLine();*/
    this_thread::sleep_for(10);
  }
  return 69420; //funny return int because of thread
}

template <class T, class U>
void moveToPoint(T x, U y, int finalAngle = 0) {
  float kPx = 4.5, kPy = 4.5, kPt = .6;
  float kDx = 0, kDy = 0, kDt = 0;
  updatePositionVars();
  float xLast = x - pos.x;
  float yLast = y - pos.y;
  float tLast = finalAngle - pos.theta; 
  float motorSpeedsA[4];

  bool complete = false;

  while(!complete) {
    updatePositionVars();

    /*Calculate the x component of our movement*/
    float xError = x - pos.x;
    float xDer = xError - xLast;
    float xLast = xError;
    float xComp = kPx*xError + kDx*xDer;
    /*End x component calculation              */

    /*Calculate the y component of our movement*/
    float yError = y - pos.y;
    float yDer = yError - yLast;
    float yLast = yError;
    float yComp = kPy*yError + kDy*yDer;
    /*End y component calculation              */

    /*Calculate turn component of our movement */
    float tError = finalAngle - pos.theta;
    float tDer = tError - tLast;
    float tLast = tError;
    float tComp = (kPt*tError) + (kDt*tDer); 
    /*End turn component calculation           */

    float targetTheta = datan2((float)(x - pos.x), (float)(y - pos.y)); 
    float tDist = pos.theta - targetTheta;

    xComp *= dcos(tDist) * fabs(xComp);
    yComp *= dsin(tDist) * fabs(yComp);

    motorSpeedsA[0] = xComp - yComp + tComp;
    motorSpeedsA[1] = xComp + yComp + tComp;
    motorSpeedsA[2] = -xComp - yComp + tComp;
    motorSpeedsA[3] = -xComp + yComp + tComp;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(pos.x);
    Brain.Screen.newLine();
    Brain.Screen.print(pos.y);
    Brain.Screen.newLine();
    Brain.Screen.print(xComp);
    Brain.Screen.newLine();
    Brain.Screen.print(yComp);
    Brain.Screen.newLine();
    Brain.Screen.print(7);
    Brain.Screen.newLine();
    std::cout << xComp << std::endl;
    std::cout << yComp << std::endl;
    std::cout << tComp << std::endl;
    std::cout << std::endl;
    float maxValue = MAX(fabs(motorSpeedsA[0]), fabs(motorSpeedsA[1]), fabs(motorSpeedsA[2]), fabs(motorSpeedsA[3]));
    if (maxValue > 100) {
      for (int i = 0; i <= 3; i ++) {
        motorSpeedsA[i] *= (100 / maxValue);
      }
    }
    frontLeft.spin(forward, motorSpeedsA[0], percent);
    backLeft.spin(forward, motorSpeedsA[1], percent);
    frontRight.spin(forward, motorSpeedsA[2], percent);
    backRight.spin(forward, motorSpeedsA[3], percent);    //spin the motors at their calculated speeds.
    
    if ((sqrt((xError*xError) + (yError*yError)) < 1.5) && (tComp < 2)) {
      complete = true;
      stopAllDrive(hold);
    }

    wait(10, msec);
  }
}

#endif //_ODOM_