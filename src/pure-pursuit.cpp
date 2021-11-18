#include "pure-pursuit.h"
#include "vex.h"

/* declared in header file, this is the point data structure
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
*/

std::vector<Point> path1 = {
  Point(30,30),
  Point(380,50),
  Point(300,200),
  Point(100,100)
};


/**
*This inject function takes a vector of defined waypoints and uses linear injection to insert evenly spaced points between each waypoint.
*
*/
std::vector<Point> inject(std::vector<Point> waypoints, float spacing) {
  std::vector<Point> newPoints;

  for(int i = 0; i < waypoints.size()-1; i++) {
    //find vector from one waypoint to adjacent one
    float vecX = waypoints[i+1].x - waypoints[i].x;
    float vecY = waypoints[i+1].y - waypoints[i].y;

    //magnitude of path segment we are injecting to
    float magnitude = sqrt((vecX*vecX) + (vecY*vecY));

    //amount of points to be added
    float possiblePoints = ceil(magnitude/spacing);

    //a unit vector that is used to add the distance from one point to another
    vecX = (vecX / magnitude) * spacing;
    vecY = (vecY / magnitude) * spacing;

    //add the each point to the newly generated path vector
    for(int j = 0; j < possiblePoints; j++) {
      newPoints.push_back( { waypoints[i].x + (vecX * j), waypoints[i].y + (vecY * j) } );
    }
  }

  return newPoints;
}

/**
* The smoother function takes a vector of points (should be an output of inject) and uses gradient descent to optimize the path for robot movement
* If a path takes too long to generate, it means that the algorithm does not properly converge. when this happens, just up the tolerance.
*/
std::vector<Point> smoother(std::vector<Point> points, float b, float tolerance) {
  std::vector<Point> newPath(points);
  float a = 1.0 - b;
  float change = tolerance;

  //ill be honest i have no idea whats going on here
  while(change >= tolerance) {
    change = 0.0;
    for(int i = 1; i < points.size()-1; i++) {
      float aux = newPath[i].x;
      newPath[i].x += a * (points[i].x - newPath[i].x)
                    + b * ((newPath[i-1].x + newPath[i+1].x)
                    - (2.0 * newPath[i].x));
      change += fabs(aux - newPath[i].x);

      aux = newPath[i].y;
      newPath[i].y += a * (points[i].y - newPath[i].y)
                    + b * ((newPath[i-1].y + newPath[i+1].y)
                    - (2.0 * newPath[i].y));
      change += fabs(aux - newPath[i].y);
    }
  }
  return newPath;
}


//draws a path to the brain screen using circles as nodes. nice for debugging.
void drawOnBrain(std::vector<Point> points, vex::color Color, int radius) {
  //Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(Color);
  for(int i = 0; i < points.size(); i++) {
    Brain.Screen.drawCircle(points[i].x, points[i].y, radius);
  }
}