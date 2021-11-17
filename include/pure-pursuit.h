#ifndef __PUREPURSUIT
#define __PUREPURSUIT

#include <vex.h>
#include <vector>
#include "math.h"
#include <string>
#include <iostream>


struct Point {
  float x = 0;
  float y = 0;
  float curvature = .5;
  float targetVelocity = 0;
  float distanceOnPath = 0;

  Point(float inX, float inY) {
    x = inX;
    y = inY;
  }

};

extern std::vector<Point> inject(std::vector<Point>, float);
extern void drawOnBrain(std::vector<Point>, vex::color, int);
extern std::vector<Point> path1;
extern std::vector<Point> smoother(std::vector<Point>, float, float);

#endif