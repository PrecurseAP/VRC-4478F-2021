#ifndef __ODOMETRY
#define __ODOMETRY

extern const float wheelCirc;
extern const float chassisWidth;
extern const float _2pi;
extern bool kill; // die
struct Robot {
  float x;
  float y;
  float theta;
};
extern Robot robot;
extern int tracking();
extern float angleWrap(float);

#endif