#include "vex.h"
#ifndef __PATHING
#define __PATHING

#include "math.h"
#include <tuple>

namespace pathing {
  extern struct point {
    float x;
    float y;
    bool end;
    point(float, float, bool);
  };
  extern class Path;
  extern void generateCubicBezier(Path*, point, point, point, point, int);
}

#endif