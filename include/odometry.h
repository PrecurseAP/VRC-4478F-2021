#ifndef __ODOMETRY
#define __ODOMETRY
#include "vex.h"

extern const float wheelCirc;
extern const float centerToRight;
extern const float centerToBack;
extern const float centerToLeft;
extern const float _2pi;
extern bool kill; // die
struct robotPosition {
  float x;
  float y;
  float theta;
};
extern robotPosition pose;

extern int trackingLoop();
extern float angleWrap(float);
extern void odomTurn(float, float, float);
#endif