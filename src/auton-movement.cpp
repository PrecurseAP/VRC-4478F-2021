//contains code that moves the robot during autonomous
#include "vex.h"
#include "odometry.h"
#include <vector>
#include "math.h"

template <typename T>
T angleWrap(T _theta) {
  return fmod(_theta, 360);
}

void moveRightSide(int speed) {
  mRUpper.spin(forward, speed, percent);
  mRLower.spin(forward, speed, percent);
}
void moveLeftSide(int speed) {
  mLUpper.spin(forward, speed, percent);
  mLLower.spin(forward, speed, percent);
}

void TurnMoveToPoint(float gx, float gy, int maxSpeed) {
  bool done = false;
  float integral = 0;

  //initialize the vector containing last 30 error values
  std::vector<float> prevValues;
  repeat(30) {
    prevValues.push_back(999);
  }

  //begin turing loop
  while(!done) {
    float ct = (M_PI/180) * angleWrap(450 - GPS.heading(degrees));

    float tDiff = (180/M_PI)*atan2( gy, gx ) - (180/M_PI)*ct;

    //erase oldest error value and insert the newest one
    prevValues.erase(prevValues.begin());
    prevValues.push_back(tDiff);

    if (fabs(tDiff) < 15) {
      integral += tDiff;
    } else {
      integral = 0;
    }
    float total = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      total += prevValues[i];
    }

    //check if the average error over 600 ms is below 1 degree, if yes then stop loop
    if (fabs((total/prevValues.size())) < 1) {
      done = true;
      break;
    }

    //spin motors at PI value
    moveRightSide(-tDiff*1.1 + -.001*integral);
    moveLeftSide(tDiff*1.1 + .001*integral);

    wait(20, msec);
  }

  prevValues.clear();
  
  repeat(30) {
    prevValues.push_back(999);
  }

  integral = 0;

  while(!done) {

    float cx = GPS.xPosition(inches);
    float cy = GPS.yPosition(inches);
    float ct = (M_PI/180) * angleWrap(450 - GYRO.heading(degrees));

    float distanceToGoal = hypot(gx-cx, gy-cy);//sqrt( (gx-cx)*(gx-cx) + (gy-cy)*(gy-cy) );

    prevValues.erase(prevValues.begin());
    prevValues.push_back(distanceToGoal);

    float angleToGoal = (180/M_PI)*atan2( gy, gx ) - (180/M_PI)*ct;

    if (fabs(distanceToGoal) < 15) {
      integral += distanceToGoal;
    } else {
      integral = 0;
    }

    float total = 0;

    for (float i = 0; i < prevValues.size(); i++) {
      total += prevValues[i];
    }

    if (fabs((total/prevValues.size())) < 1) {
      done = true;
      break;
    }

    moveLeftSide(distanceToGoal*1.1 + integral*0.0001 + angleToGoal);
    moveRightSide(distanceToGoal*1.1 + integral*0.0001 - angleToGoal);
  }
}