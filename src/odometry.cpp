#include "odometry.h"
#include "math.h"

const float wheelCirc = 2.75 * M_PI;
const float centerToRight = 3.13;
const float centerToBack = 2.75;
const float centerToLeft = 3.13;
const float _2pi = M_PI * 2.0;

bool kill = false;

robotPosition pose = {0.0, 0.0, 0.0};

float angleWrap(float val) {
  return fmod(val, _2pi);
}

int trackingLoop() {

  twRight.setPosition(0, degrees);
  twLeft.setPosition(0, degrees);
  twBack.setPosition(0, degrees);

  using namespace vex;

  kill = false;

  float prevL = 0;
  float prevR = 0;
  float prevB = 0;
  float prevTheta = 0;

  float h, h2; //declare variables for the change in distance in each direction (i think)

  while(!kill) {
    //initialize values

    float currL = twLeft.position(degrees);
    float currR = twRight.position(degrees);
    float currB = twBack.position(degrees);
    float currTheta = GYRO.heading(degrees) * M_PI / 180;
    //get current values of the encoders in inches

    //float deltaLeft = (currL - prevL) / 360.0  * wheelCirc;
    float deltaRight = (currR - prevR) / 360.0  * wheelCirc;
    float deltaBack = (currB - prevB) / 360.0  * wheelCirc;
    //find change in distance since last code execution

    //float deltaTheta = (deltaLeft - deltaRight) / 6.26; //change in orientation, remember this is always in radians regardless of the units of anything else
    float deltaTheta = angleWrap(currTheta - prevTheta);
    prevTheta = currTheta;

    if (deltaTheta != 0) {
      float sinDT = sin(deltaTheta / 2.0);
      h = (((deltaRight / deltaTheta) + centerToRight) * sinDT) * 2.0;  //calculate new local offset if the robot performs a turning motion
      h2 = (((deltaBack / deltaTheta) + centerToBack) * sinDT) * 2.0;
    } else {
      h = deltaRight;
      h2 = deltaBack; //if the robot doesnt turn, no calculation is needed
    }

    float averageOrientation = (deltaTheta / 2.0) + pose.theta; //calculate the average orientation
    
    float cosAO = cos(averageOrientation);
    float sinAO = sin(averageOrientation); 
    //calculate local (relative) changes in position

    prevL = currL;
    prevR = currR;
    prevB = currB;
    //store values for next iteration

    pose.x += (h * sinAO) + (h2 * cosAO);
    pose.y += (h * cosAO) + (h2 * -sinAO);
    pose.theta += deltaTheta;
    //calculate and store the new absolute pose of the robot

    wait(20, msec);
  }
  return 1;
}