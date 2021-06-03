#include "vex.h"
#ifndef __PATHING
#define __PATHING

#include "math.h"
#include <tuple>
#include "util.h"

namespace pathing {
  extern struct Point {
    float x;
    float y;
    bool end;
    Point(float, float, bool);
  };
  extern class Path;
  extern void generateCubicBezier(Path*, Point, Point, Point, Point, int);
}

#endif