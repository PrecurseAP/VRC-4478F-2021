#ifndef __ODOMETRY
#define ODOMETRY
#include "vex.h"

namespace odom {
  extern const float wheelCirc;
  extern const float chassisWidth;
  extern const float _2pi;
  extern bool kill;
  extern struct Position {
    float x;
    float y;
    float theta; 
  };
  extern int trackingLoop(Position*); 
  extern float angleWrap(float);
}
#endif