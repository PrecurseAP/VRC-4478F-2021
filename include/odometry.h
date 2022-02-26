#ifndef __ODOMETRY
#define ODOMETRY
#include "vex.h"

extern const float wheelCirc;
extern const float chassisWidth;
extern const float _2pi;
extern bool kill; // die
struct robotPosition {
  float x;
  float y;
  float theta;
};
extern robotPosition pose;
extern int trackingLoop(robotPosition *);
extern float angleWrap(float);

#endif