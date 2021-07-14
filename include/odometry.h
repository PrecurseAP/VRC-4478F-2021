#ifndef __ODOMETRY
#define ODOMETRY
#include "vex.h"

extern const float wheelCirc;
extern const float chassisWidth;
extern const float _2pi;
extern bool kill; // die
struct Position {
  float x;
  float y;
  float theta;
};
extern Position pose;
extern int trackingLoop(Position *);
extern float angleWrap(float);

#endif