//i like how the name of the file is a palindrome
#include <tuple>
#include "ppc.h"
#include "odometry.h"

float asyncSpeedMod = 1; //global speed mod for async event listener

float lookahead = 18.0;
float lookaheadKF = .5;
float kTurn = 10;

float calcAngleErrorRad(float a, float b) {
  /**
  * calculate the error between two angles, returns the shortest path
  * calculates error FROM A TO B (positive means that you have to turn right, negative means you turn left)
  * this is in radians
  */
  if ((b - a) > M_PI) {
    return b - a - _2pi;
  } else if ((b - a) < -M_PI) {
    return b - a + _2pi;
  } else {
    return b - a;
  }
}

void loop(Path* path) {
  bool done = false;
  while(!done) {
    Point closest;
    float dClosest;

    std::tie(closest, dClosest) = path->nearestPointAndDistance(Point(pose.x, pose.y));

    if (closest.end == true) {
      done = dClosest < 2;
    }

    float newLookahead = lookahead - (dClosest * lookaheadKF);
    newLookahead = (newLookahead < 0) ? 1 : newLookahead;

    Point target = path->getPointAtIndex(closest.ind);

    float distanceToLookaheadPoint = sqrt(sq(target.x - pose.x) + sq(target.y - pose.y));

    float forwardPower = distanceToLookaheadPoint * 10;

    float targetBearing = atan2(target.x - pose.x, target.y - pose.y);

    while (targetBearing < 0) {
      targetBearing += _2pi;
    }

    float currentBearing = atan2(sin(pose.theta), cos(pose.theta));

    float angleError = calcAngleErrorRad(currentBearing, targetBearing) * kTurn;

    mLFront.spin(forward, forwardPower + angleError, percent);
    mRFront.spin(forward, forwardPower - angleError, percent);
    mLBack.spin(forward, forwardPower + angleError, percent);
    mRBack.spin(forward, forwardPower - angleError, percent);
  }
}