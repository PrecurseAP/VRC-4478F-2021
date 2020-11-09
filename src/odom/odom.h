#ifndef _ODOM_H_
#define _ODOM_H_
#include <iostream> //yo my slime, i know you dont really know mandem like that, but i was wondering if, like, i could purchase summin styl fam. just a bit of grub my drilla
#include "vex.h"
#include "../src/aidenmath/custommath.h"
#include <vex_timer.h>

timer distanceCheck;
timer turnCheck;


float rX = 0;
float rY = 0;

const float wheelCirc = 2.75 * M_PI; //circumference of a 2.75" omni wheel, the ones we use for tracking wheels. This is used to calculate the distance a wheel has traveled.

const float centerToLeft = 6.0; 
const float centerToRight = 6.0; //all units in inches
const float centerToBack = 4.75;

double deltaLeft, deltaRight, deltaBack;

float leftEnc, rightEnc, backEnc;
float prevLeft = 0, prevRight = 0, prevBack = 0;

float theta = 0; //variables for the gyro angle
float prevTheta = 0;

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

    float hyp, hyp2;

    float deltaTheta = (deltaLeft - deltaRight) / 12.0;

    if (deltaTheta) {
      float sinDT = sin(deltaTheta / 2.0);
      hyp = (((deltaRight / deltaTheta) + centerToRight) * sinDT) * 2.0;
      hyp2 = (((deltaBack / deltaTheta) + centerToBack) * sinDT) * 2.0;
    } else {
      hyp = deltaRight;
      hyp2 = deltaBack;
    }

    float averageOrientation = (deltaTheta / 2.0) + prevTheta;
    float cosAO = cos(averageOrientation);
    float sinAO = sin(averageOrientation);

    rY += hyp * cosAO;
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

template <class T>
void moveToPoint(T x, T y, int finalAngle = 0) {

}

#endif //_ODOM_