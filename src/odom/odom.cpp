#include "../src/odom/odom.h"
#include <iostream> //yo my slime, i know you dont really know mandem like that, but i was wondering if, like, i could purchase summin styl fam. just a bit of grub my drilla
#include "vex.h"
#include "../src/aidenmath/custommath.h"
#include <vex_timer.h>
#include "../src/drive/drive.h"

template <class T>
void debug(T arg) {
  Brain.Screen.print(arg);
  Brain.Screen.newLine();
}

void initDebug() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
}

struct robotPosition {
  float x;
  float y;
  float theta;
} pos;

thread ODOM;

float rX = 0, rY = 0, averageOrientation, prevTheta = 0;
float wheelCirc = 2.75 * M_PI; //circumference of a 2.75" omni wheel, the ones we use for tracking wheels. This is used to calculate the distance a wheel has traveled.

float centerToLeft = 6.0; 
float centerToRight = 6.0; //all units in inches
float centerToBack = 4.75;

float deltaLeft, deltaRight, deltaBack;

float leftEnc, rightEnc, backEnc;
float prevLeft = 0, prevRight = 0, prevBack = 0;

bool atPoint = false;

float kPx = 4.5, kPy = 4.5, kPt = .6;
float kDx = 0, kDy = 0, kDt = 0;

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
    
    this_thread::sleep_for(10);
  }
  return 69420; //funny return int because of thread
}

void swerve(float x, float y, int finalAngle = 0) {

  updatePositionVars();
  float xLast = x - pos.x;
  float yLast = y - pos.y;
  float tLast = finalAngle - pos.theta; 
  float mS[4];

  atPoint = false;

  while(!atPoint) {
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

    float targetTheta = datan2((x - pos.x), (y - pos.y)); 
    float tDist = pos.theta - targetTheta;

    xComp *= dcos(tDist) /** fabs(xComp)*/;
    yComp *= dsin(tDist) /** fabs(yComp)*/;

    mS[0] = xComp - yComp + tComp;
    mS[1] = xComp + yComp + tComp;
    mS[2] = -xComp - yComp + tComp;
    mS[3] = -xComp + yComp + tComp;

    float maxValue = MAX(fabs(mS[0]), fabs(mS[1]), fabs(mS[2]), fabs(mS[3]));
    if (maxValue > 100) {
      for (int i = 0; i <= 3; i ++) {
        mS[i] *= (100 / maxValue);
      }
    }

    initDebug();
    debug(mS[0]);
    debug(mS[1]);
    debug(mS[2]);
    debug(mS[3]);

    frontLeft.spin(forward, mS[0], percent);
    backLeft.spin(forward, mS[1], percent);
    frontRight.spin(forward, mS[2], percent);
    backRight.spin(forward, mS[3], percent);    //spin the motors at their calculated speeds.
    
    if ((sqrt((xError*xError) + (yError*yError)) < 1.5) && (tComp < 2)) {
      atPoint = true;
      stopAllDrive(hold);
    }

    wait(10, msec);
  }
}