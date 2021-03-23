#include "odom.h"
#include <iostream> //yo my slime, i know you dont really know mandem like that, but i was wondering if, like, i could purchase summin styl fam. just a bit of grub my drilla
#include "vex.h"
#include "custommath.h"
#include <vex_timer.h>

thread ODOM;

float rX = 0, rY = 0, averageOrientation, globalOrientation = 0;
float wheelCirc = 2.75 * M_PI; //circumference of a 2.75" omni wheel, the ones we use for tracking wheels. This is used to calculate the distance a wheel has traveled.

float centerToLeft = 4.90; 
float centerToRight = 4.90; //all units in inches
float centerToBack = 6.10;

float deltaLeft, deltaRight, deltaBack;

float leftEnc, rightEnc, backEnc;
float prevLeft = 0, prevRight = 0, prevBack = 0;

bool atPoint = false;

struct robotPosition {
  float x;
  float y;
  float thetaDeg;
  float thetaRad;
} pos;

void resetPos() {
  pos.x = 0;
  pos.y = 0;
  pos.thetaDeg = 0;
  pos.thetaRad = 0;
  rX = 0;
  rY = 0;
  globalOrientation = 0;
  averageOrientation = 0;
}

void updatePositionVars() {
  /**
   * This function is used to store the robot's pose in variables separated from the asynchronous tracking function
   */
  pos.x = rX;
  pos.y = rY;
  pos.thetaDeg = globalOrientation * radToDeg;
  pos.thetaRad = globalOrientation;
  //pos.gyroheadRad = GYRO.heading(degrees) * degToRad;
}

int tracking() {
  /**
   * The goal of this function is to use values from 3 tracking wheels to track x and y coordinates of the robot on the field during autonomous.
   */
  leftEncoder.resetPosition();
  rightEncoder.resetPosition(); //reset the encoders so that there is no error upon starting the tracking code.
  backEncoder.resetPosition();

  while(true) {
    leftEnc = leftEncoder.position(degrees);  //This is negative because the left tracking wheel is reversed relative the the right one.
    rightEnc = rightEncoder.position(degrees); //store encoder values
    backEnc = backEncoder.position(degrees);  //this is negative so that the direction of tracking is reversed, otherwise +x would be a negative value

    deltaLeft = ((leftEnc - prevLeft)/360) * wheelCirc;
    deltaRight = ((rightEnc - prevRight)/360) * wheelCirc; //calculate distance traveled by each tracking wheel measured in inches
    deltaBack = ((backEnc - prevBack)/360) * wheelCirc;

    prevLeft = leftEnc;
    prevRight = rightEnc; //store previous values for next delta calculation
    prevBack = backEnc;

    float h, h2; //declare variables for the change in distance in each direction (i think)

    float deltaTheta = (deltaLeft - deltaRight) / 9.8; //change in orientation, remember this is always in radians regardless of the units of anything else

    if (deltaTheta) {
      float sinDT = sin(deltaTheta / 2.0);
      h = (((deltaRight / deltaTheta) + centerToRight) * sinDT) * 2.0;  //calculate new local offset if the robot performs a turning motion
      h2 = (((deltaBack / deltaTheta) + centerToBack) * sinDT) * 2.0;
    } else {
      h = deltaRight;
      h2 = deltaBack; //if the robot doesnt turn, no calculation is needed
    }

    float averageOrientation = (deltaTheta / 2.0) + globalOrientation; //calculate the average orientation
    
    float cosAO = cos(averageOrientation);
    float sinAO = sin(averageOrientation); 

    rY += (h * cosAO) + (h2 * -sinAO);    //calculate the final change in position by rotating the local offset point by the average orientation
    rX += (h * sinAO) + (h2 * cosAO);

    globalOrientation += deltaTheta; //update global position variable
    
    this_thread::sleep_for(10); //add a 10ms delay between each run of the loop
  }
  return 69420; //return int because threads require an int return
}

void moveToPoint(float gx, float gy, float theta) {
  atPoint = false;
  float s1_angle;

  //stage 1: turn towards point
  //this switch statement finds the angle of the goal point to the vertical from 0 - 360 degrees
  float dx = gx - pos.x;
  float dy = gy - pos.y;

  switch((int)sign(dx)) {
    case 1:
      switch((int)sign(dy)) {
        case 1: //quad 1
          s1_angle = atan2(dx, dy);
        break;
        case -1: //quad 2
          s1_angle = M_PI - atan2(dx, dy);
        break;
      }
    break;
    case -1:
      switch((int)sign(dy)) {
        case 1: //quad 4
          s1_angle = (2 * M_PI) - atan2(dx, dy);
        break;
        case -1: //quad 3
          s1_angle = M_PI + atan2(dx, dy);
        break;
      }
    break;
  }
  s1_angle *= (180/M_PI);
}