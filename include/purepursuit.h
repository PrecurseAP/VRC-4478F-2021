#ifndef __PUREPURSUIT
#define __PUREPURSUIT

#include <vex.h>
#include <vector>
#include "math.h"
#include <string>
#include <iostream>
#include "odometry.h"


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

extern std::vector<Point> calculateDistances(std::vector<Point>);

extern std::vector<Point> calculateCurvatures(std::vector<Point>);

extern std::vector<Point> calculateVelocities(std::vector<Point>, float, float, float, float);

extern float robotX, robotY, robotTheta, robotTrackWidth;

extern float distanceFormula(Point, Point);

extern int purePursuit(std::vector<Point>, float, robotPosition*);

extern float clip(float, float, float);

struct purePursuitData {
  int closestPoint;
  int prevClosestPoint = 0;
  Point lookaheadPoint = Point(0,0);
  float prevLookahead;

  purePursuitData(Point WAP) {
    closestPoint = 0;
    prevClosestPoint = 0;
    lookaheadPoint = WAP;
    prevLookahead = 0.0;
  }

};

#endif