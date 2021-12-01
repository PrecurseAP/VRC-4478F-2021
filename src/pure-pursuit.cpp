#include "pure-pursuit.h"
#include "vex.h"
#include <algorithm>

template <typename G> 
int sgn2(G val) { //SIGNUM
    return (G(0) < val) - (val < G(0));
}

float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

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
  Point(0, 0),
  Point(0, 20),
  Point(20, 20),
  Point(20, 0)
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

/**
* this is the function that accepts a vector of points (should be ouput from smoother) and adds the distance between them
*/
std::vector<Point> calculateDistances(std::vector<Point> path) {
  std::vector<Point> newPath(path);
  
  newPath[0].distanceOnPath = 0;
  
  
  for(int i = 1; i < path.size(); i++) {
    newPath[i].distanceOnPath = newPath[i-1].distanceOnPath + sqrt((newPath[i].x - newPath[i-1].x)*(newPath[i].x - newPath[i-1].x)
                                    + (newPath[i].y - newPath[i-1].y)*(newPath[i].y - newPath[i-1].y));
  }
  return newPath;
}

/**
* This is the function that calculates the curvature at each point, input should come from calculateDistances
*/
std::vector<Point> calculateCurvatures(std::vector<Point> path) {
  std::vector<Point> newPath(path);
  
  for (int i = 1; i < path.size()-1; i++) {
    float x1 = newPath[i].x;
    float y1 = newPath[i].y;
    float x2 = newPath[i-1].x;
    float y2 = newPath[i-1].y;
    float x3 = newPath[i+1].x;
    float y3 = newPath[i+1].y;

      if (x1 == x2) {
    x1 += 0.001; 
  }
    
    float k1 = 0.5 * (x1*x1 + y1*y1 - x2*x2 - y2*y2) / (x1-x2);
    float k2 = (y1 - y2) / (x1 - x2);
    float b = 0.5 * (x2*x2 - 2*x2*k1 + y2*y2 - x3*x3 + 2*x3*k1 - y3*y3) / (x3*k2 - y3 + y2 - x2*k2);
    float a = k1 - k2*b;
    float r = sqrt( (x1-a)*(x1-a) + (y1-b)*(y1-b) );
    float c = 1/r;
    newPath[i].curvature = c; 
  }
  return newPath;
}

//maximum velocity that the robot can move at is 4.72 ft/sec or 56.7 in/s (this is for 5:3 speed drive with 3.25" omnis)
//normal 200 rpm drive with 4" omnis is 41.8 in/sec, or 3.49 ft/sec
std::vector<Point> calculateVelocities(std::vector<Point> path, float pathMaxVel, float k, float a, float k2) {
  std::vector<Point> newPath(path);
  
  newPath[0].targetVelocity = pathMaxVel;
  newPath[newPath.size()-1].targetVelocity = 0;

  for(int i = 1; i < newPath.size()-1; i++) {
    newPath[i].targetVelocity = std::min(pathMaxVel, k/(k2*newPath[i].curvature));
  }

  for(int i = newPath.size()-2; i >= 0; i--) {
    float dis = sqrt((newPath[i].x - newPath[i+1].x)*(newPath[i].x - newPath[i+1].x)
                + (newPath[i].y - newPath[i+1].y)*(newPath[i].y - newPath[i+1].y));
    newPath[i].targetVelocity = std::min(newPath[i].targetVelocity, 
                                (float)sqrt((newPath[i+1].targetVelocity)*(newPath[i+1].targetVelocity) + 2.0*a*dis));
  }

  return newPath;
}

// in header file
/* struct purePursuitData {
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

};*/

float robotX = 0; //placeholders for when we actually have odometry.
float robotY = 0;
float robotTheta = 0;
float robotTrackWidth = 12;

float distanceFormula(Point a, Point b) {
  return sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
}

int purePursuit(std::vector<Point> path, float lookahead) {
  purePursuitData data = purePursuitData(path[0]);

  float prevLimitedVelocity = 0;
  float vOutput = 0;

  float maxRate = 50;

  int start_time = Brain.timer(msec);
  int prevTime = 0;

  bool done = false;
  while(!done) {

    robotX = round(GPS.xPosition(inches));
    robotY = round(GPS.yPosition(inches));
    robotTheta = round((180/M_PI)*atan2(-sin((M_PI/180)*GYRO.heading(degrees)), cos((M_PI/180)*GYRO.heading(degrees))));

    //find closest point
    for (int i = data.closestPoint; i < path.size(); i++) {
      float distanceToPoint = distanceFormula(Point(robotX, robotY), path[i]);
      if (distanceToPoint < distanceFormula(Point(robotX, robotY), path[data.closestPoint])) {
        data.closestPoint = i; 
      }
    }

    //calculate lookahead point
    for (int i = floor(data.prevLookahead); i < path.size()-1; i++) {
      float dx = path[i+1].x - path[i].x;
      float dy = path[i+1].y - path[i].y;

      float fx = path[i].x - robotX;
      float fy = path[i].y - robotY;

      float a = dx*dx + dy*dy;
      float b = 2.0 * ((fx*dx) + (fy*dy));
      float c = (fx*fx + fy*fy) - (lookahead*lookahead);
      float discriminant = b*b - 4*a*c;

      if (discriminant < 0) {
        //no intersection
      } else {
        discriminant = sqrt(discriminant);
        
        float t1 = (-b - discriminant) / (2*a);
        float t2 = (-b + discriminant) / (2*a);

        if (t1 >= 0 && t1 <= 1) {
          float intersectionX = path[i].x + (t1*dx);
          float intersectionY = path[i].y + (t1*dy);

          float fracIndex = i + t1;

          if (fracIndex > data.prevLookahead) {
            data.lookaheadPoint = Point(intersectionX, intersectionY);
            data.prevLookahead = fracIndex;
          }
        }
        if (t2 >= 0 && t2 <= 1) {
          float intersectionX = path[i].x + (t2*dx);
          float intersectionY = path[i].y + (t2*dy);

          float fracIndex = i + t2;

          if (fracIndex > data.prevLookahead) {
            data.lookaheadPoint = Point(intersectionX, intersectionY);
            data.prevLookahead = fracIndex;
          }
        }
      }
      //otherwise no intersection
    }

    //old method
    /*float xFromRobotToLookahead = fabs((-tan(robotTheta)*data.lookaheadPoint.x)
                                        + data.lookaheadPoint.y 
                                        + (tan(robotTheta)*robotX - robotY))
                                        / sqrt((-tan(robotTheta))*(-tan(robotTheta)) + 1);*/

    //new method
    float cosHeading = cos((M_PI/180)*robotTheta);
    float sinHeading = sin((M_PI/180)*robotTheta);

    float xFromRobotToLookahead = (data.lookaheadPoint.x-robotX)*cosHeading + (data.lookaheadPoint.y-robotY)*sinHeading;
    float yFromRobotToLookahead = -(data.lookaheadPoint.x-robotX)*sinHeading + (data.lookaheadPoint.y-robotY)*cosHeading;

    float lookaheadCurvature = (2.0*xFromRobotToLookahead) / (float)(lookahead*lookahead);

    float side = -sgn2(sinHeading*xFromRobotToLookahead - cosHeading*yFromRobotToLookahead);

    float signedCurvature = lookaheadCurvature * side; //that took a while

    //begin wheel velocity calculation

    int currentTime = Brain.timer(msec);
    int delta_time = currentTime - prevTime;
    prevTime = currentTime;

    if (path[data.closestPoint].targetVelocity > prevLimitedVelocity) {
      maxRate = 25;
    } else {
      maxRate = 500;
    }

    float maxChange = (delta_time/1000) * maxRate;
    vOutput += clip(path[data.closestPoint].targetVelocity - prevLimitedVelocity, -maxChange, maxChange);
    prevLimitedVelocity = vOutput;

    float targetLeftVelocity = path[data.closestPoint].targetVelocity * (2 + signedCurvature*robotTrackWidth) / 2;
    float targetRightVelocity = path[data.closestPoint].targetVelocity * (2 - signedCurvature*robotTrackWidth) / 2;

    float targetLeftPower = ((targetLeftVelocity / (4 * M_PI)) * 60);
    float targetRightPower = ((targetRightVelocity / (4 * M_PI)) * 60);

    std::cout << path[data.closestPoint].targetVelocity << std::endl;
    /*Brain.Screen.clearScreen();
    drawOnBrain(path, color::red, 2);
    Brain.Screen.setPenColor(color::green);
    Brain.Screen.drawCircle(path[data.closestPoint].x+40, path[data.closestPoint].y+40, 3);
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.drawCircle(data.lookaheadPoint.x+40, data.lookaheadPoint.y+40, 2);
    Brain.Screen.setPenColor(color::yellow);
    Brain.Screen.drawCircle(robotX+40, robotY+40, 1);

    float X1 = robotX;
    float Y1 = robotY;
    float X2 = data.lookaheadPoint.x;
    float Y2 = data.lookaheadPoint.y;

    float xa = (X2-X1)/2;
    float ya = (Y2-Y1)/2;

    float X0 = X1+xa;
    float Y0 = Y1+ya;

    float a = sqrt( (xa*xa) + (ya*ya) );

    float r = 0;

    if (signedCurvature != 0) {
      r = 1/signedCurvature;
    } else {
      r = 50;
    }

    float b = side*sqrt(fabs(r*r) - a*a);

    float X3 = X0 + ((b*ya)/a);
    float Y3 = Y0 - ((b*xa)/a);
    Brain.Screen.setPenColor(color::blue);
    Brain.Screen.drawCircle(X3, Y3, r, false);*/
  

    mLUpper.spin(forward, -targetRightPower, rpm);
    mLLower.spin(forward, -targetRightPower, rpm);
    mRUpper.spin(forward, -targetLeftPower, rpm);
    mRLower.spin(forward, -targetLeftPower, rpm);

    wait(20, msec);
  }
  return 0;
}

//draws a path to the brain screen using circles as nodes. nice for debugging.
void drawOnBrain(std::vector<Point> points, vex::color Color, int radius) {
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(Color);
  for(int i = 0; i < points.size(); i++) {
    Brain.Screen.drawCircle(points[i].x+40, points[i].y+40, radius);
  }
}
