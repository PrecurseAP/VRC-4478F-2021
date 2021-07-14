#include "vex.h"
#ifndef __PATHING
#define __PATHING

#include "math.h"
#include <tuple>
#include "util.h"

namespace pathing {
  enum eventType {
    accelerate, intakes, lift, escape, empty  
  };
  struct Event {
    eventType type;
    int v1;
    int v2;
    bool sequential;
    Event(eventType, int, int, bool);
    Event(eventType, int, bool);
    Event(eventType, bool);
  };
  struct Point {
    float x;
    float y;
    int ind;
    bool end;
    Event e;
    //dont tell me what to do
    Point(void);
    Point(float, float);
    Point(float, float, int); 
    Point(float, float, bool);
    Point(float, float, int, bool);
  };
  class Path {
    private:
      int resolution;
      Point p[];
    public:
      Path(Point, Point, Point, Point, int);
      std::tuple<Point, float> nearestPointAndDistance(Point);
      void setPointAtIndex(int, Point);
      Point getPointAtIndex(int);
  };
  extern void generateCubicBezier(Path*, Point, Point, Point, Point, int);

  extern int eventListener(Path*);
}

#endif