#include "../src/odom/odom.h"
#include <iostream> //yo my slime, i know you dont really know mandem like that, but i was wondering if, like, i could purchase summin styl fam. just a bit of grub my drilla
#include "vex.h"
#include "../src/aidenmath/custommath.h"
#include <vex_timer.h>
#include "../src/drive/drive.h"

thread ODOM;

float rX = 0, rY = 0, averageOrientation, globalOrientation = 0;
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
  leftEncoder.setPosition(0, degrees);
  rightEncoder.setPosition(0, degrees); //reset the encoders so that there is no error upon starting the tracking code.
  backEncoder.setPosition(0, degrees);

  while(true) {
    leftEnc = -leftEncoder.position(degrees);  //This is negative because the left tracking wheel is reversed relative the the right one.
    rightEnc = rightEncoder.position(degrees); //store encoder values
    backEnc = -backEncoder.position(degrees);  //this is negative so that the direction of tracking is reversed, otherwise +x would be a negative value

    deltaLeft = ((leftEnc - prevLeft)/360) * wheelCirc;
    deltaRight = ((rightEnc - prevRight)/360) * wheelCirc; //calculate distance traveled by each tracking wheel measured in inches
    deltaBack = ((backEnc - prevBack)/360) * wheelCirc;

    prevLeft = leftEnc;
    prevRight = rightEnc; //store previous values for next delta calculation
    prevBack = backEnc;

    float h, h2; //declare variables for the change in distance in each direction (i think)

    float deltaTheta = (deltaLeft - deltaRight) / 12.0; //change in orientation, remember this is always in radians regardless of the units of anything else

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
    
    this_thread::sleep_for(10); //add a 10ms delay between tracking code executions
  }
  return 69420; //funny return int because threads require an int return
}

void swerve(float x, float y, int finalAngle = 0) {
  /**
   * This function is the primary movement function for autonomous.
   * It works by simulating controller inputs based on the error in x, y, and orientation.
   */
  updatePositionVars();
  float xLast = x - pos.x;
  float yLast = y - pos.y;
  float tLast = finalAngle - pos.thetaRad; 
  float mS[4];
  float normalizer;

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
    float tError = finalAngle - pos.thetaDeg;
    float tDer = tError - tLast;
    float tLast = tError;
    float tComp = (kPt*tError) + (kDt*tDer); 
    /*End turn component calculation           */

    float targetTheta = atan2f((x - pos.x), (y - pos.y)); 
    float tDist = pos.thetaRad + targetTheta;

    xComp *= cos(tDist) /** fabs(xComp)*/;
    yComp *= sin(tDist) /** fabs(yComp)*/;

    mS[0] = xComp - yComp + tComp;
    mS[1] = xComp + yComp + tComp;
    mS[2] = -xComp - yComp + tComp;
    mS[3] = -xComp + yComp + tComp;

    float maxAxis = MAX(fabs(xComp), fabs(yComp), fabs(tComp)); //Find the maximum input given by the controller's axes and the angle corrector
    float maxOutput = MAX(fabs(mS[0]), fabs(mS[1]), fabs(mSpd[2]), fabs(mSpd[3])); //Find the maximum output that the drive program has calculated

    if (maxOutput == 0 || maxAxis == 0) {
      normalizer = 0; //Prevent the undefined value for normalizer
    } else {
      normalizer = maxAxis / maxOutput; //calculate normalizer
    }

    for (int i = 0; i <= 3; i++) {
      mS[i] *= normalizer; //caps motor speeds to the greatest input without losing the ratio between each speed, so as to not warp the direction of movement too much.
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