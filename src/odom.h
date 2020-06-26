#ifndef _ODOM_
#define _ODOM_

#include "vex.h"
#include "math.h"
#include "custommath.h"
#include "trig.h"

float robotX;
float robotY;

//QUADRANTS USED IN THIS CODE
//      |
//   2  |  1
//-------------
//   4  |  3
//      |

int getQuadrantOfPoint(float inX, float inY)  {
  switch(sign(inX)) {
    case -1:
      return sign(inY) == -1 ? 4 : 2;
    break;
    case 1:
      return sign(inY) == -1 ? 3 : 1; 
    break;
    default: return 0; break;
  }
}    

float distanceToPoint(float inX, float inY) {
  return sqrt( (inX - robotX) + (inY - robotY) );
}

void moveRobotToPoint(float gX, float gY, int gAngle, int speed) { //g means goal

double dist = distanceToPoint(gX, gY);
double gyroAngle = GYRO.heading(degrees);



}

#endif //_ODOM_