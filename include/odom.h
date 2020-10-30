#ifndef _ODOM_H_
#define _ODOM_H_
#include <iostream>
//yo my slime, i know you dont really know mandem like that, but i was wondering if, like, i could purchase summin styl fam. just a bit of grub my drilla


#include "vex.h"
#include "math.h"
#include "custommath.h"
#include "trig.h"

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
    leftEnc = -leftEncoder.position(rev);
    rightEnc = rightEncoder.position(rev); //store encoder values
    backEnc = -backEncoder.position(rev);

    deltaLeft = (leftEnc - prevLeft) * wheelCirc;
    deltaRight = (rightEnc - prevRight) * wheelCirc; //calculate distance traveled by each tracking wheel represented in inches
    deltaBack = (backEnc - prevBack) * wheelCirc;

    prevLeft = leftEnc;
    prevRight = rightEnc; //store previous values
    prevBack = backEnc;

    theta += radToDeg*((deltaLeft - deltaRight)/12.0); //calculate new orientation angle

    float deltaTheta = theta - prevTheta; //calculate change in angle since last cycle

    if ((deltaTheta == 0) || ((deltaLeft - deltaRight) == 0)) { //calculate local offset. if there is no change in orientation, it is just the tracking wheel distances.
      localOffset[0] = deltaBack;    
      localOffset[1] = deltaRight;
    } else {                                                            //if the orientation has changed, then the local offset is calculated.  
      float temp = 2 * dsin(theta/2);      
      localOffset[0] = temp * ((deltaBack / deltaTheta) + centerToBack);
      localOffset[1] = temp * ((deltaRight / deltaTheta) + centerToRight);
    }

    float averageOrientation = prevTheta + (deltaTheta/2); //calculate average orientation, which is the difference between the local and global offsets.
    prevTheta = theta; //store gyro angle for use in next cycle

    globalOffset[0] = (dcos(-averageOrientation)*localOffset[0]) - (dsin(-averageOrientation)*localOffset[1]); //calculate global offset vector
    globalOffset[1] = (dcos(-averageOrientation)*localOffset[1]) + (dsin(-averageOrientation)*localOffset[0]);

    rX += globalOffset[0]; //calculate position by compounding the global offset every cycle. the sum of all global offset vectors = current position.
    rY += globalOffset[1];
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(rX);
    Brain.Screen.newLine();
    Brain.Screen.print(rY);
    Brain.Screen.newLine();
    //Brain.Screen.print(backEnc);
    Brain.Screen.newLine();
    this_thread::sleep_for(10);
  }
  return 69420; //funny return int because of thread
}
template <class T>
void travelToPoint(T x, T y, int dspeed = 75, int tspeed = 50, int finalAngle = 420) {
  int quadrantAdd;
  float northAngle;
  float xdif = x - rX;
  float ydif = y - rY;

  float distanceToGoal = sqrt((xdif*xdif) + (ydif*ydif));

  switch(sign(xdif)) {
    case 1:
      quadrantAdd = (sign(ydif) == 1) ? 0 : 90;
      northAngle = (sign(ydif) == 1) ? datan(xdif/ydif) : datan(ydif/xdif);
    break;
    case -1:
      quadrantAdd = (sign(ydif) == 1) ? 270 : 180;
      northAngle = (sign(ydif) == 1) ? datan(ydif/xdif) : datan(xdif/ydif);
    break;
    default:
      quadrantAdd = 0;
      northAngle = 0; //these should never execute. sign() returns only 1 or -1
    break;
  }
  northAngle += quadrantAdd;


}

#endif //_ODOM_