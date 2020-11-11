#ifndef __ODOM__
#define __ODOM__
#include <iostream> //yo my slime, i know you dont really know mandem like that, but i was wondering if, like, i could purchase summin styl fam. just a bit of grub my drilla
#include "vex.h"
#include "../src/aidenmath/custommath.h"
#include <vex_timer.h>

timer distanceCheck;
timer turnCheck;

struct robotPosition {
  float x;
  float y;
  float theta;
} pos;

float rX = 0;
float rY = 0;

float prevTheta = 0; //variables for the gyro angle

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



#endif //_ODOM_