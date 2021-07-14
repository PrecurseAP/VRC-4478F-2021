//i like how the name of the file is a palindrome
#include <tuple>
#include "ppc.h"
#include "odometry.h"

using namespace ppc;

float ppc::asyncSpeedMod = 1; //global speed mod for async event listener

float ppc::lookahead = 18.0;
float ppc::lookaheadKF = .5;
float ppc::kTurn = 10;

float calcAngleErrorRad(float a, float b) {
  /**
  * calculate the error between two angles, returns the shortest path
  * calculates error FROM A TO B (positive means that you have to turn right, negative means you turn left)
  * this is in radians
  */
  if ((b - a) > M_PI) {
    return b - a - odom::_2pi;
  } else if ((b - a) < -M_PI) {
    return b - a + odom::_2pi;
  } else {
    return b - a;
  }
}

void ppc::loop(pathing::Path* path) {
  bool done = false;
  while(!done) {
    pathing::Point closest;
    float dClosest;

    std::tie(closest, dClosest) = path->pathing::nearestPointAndDistance(pathing::Point(odom::pose.x, odom::pose.y));

    if (closest.end == true) {
      done = dClosest < 2;
    }

    float newLookahead = lookahead - (dClosest * lookaheadKF);
    newLookahead = (newLookahead < 0) ? 1 : newLookahead;

    pathing::Point target = path->getPointAtIndex(closest.ind);

    float distanceToLookaheadPoint = sqrt(sq(target.x - odom::pose.x) + sq(target.y - odom::pose.y));

    float forwardPower = distanceToLookaheadPoint * 10;

    float targetBearing = atan2(target.x - odom::pose.x, target.y - odom::pose.y);

    while (targetBearing < 0) {
      targetBearing += odom::_2pi;
    }

    float currentBearing = atan2(sin(odom::pose.theta), cos(odom::pose.theta));

    float angleError = calcAngleErrorRad(currentBearing, targetBearing) * kTurn;

    mLFront.spin(forward, forwardPower + angleError, percent);
    mRFront.spin(forward, forwardPower - angleError, percent);
    mLBack.spin(forward, forwardPower + angleError, percent);
    mRBack.spin(forward, forwardPower - angleError, percent);
  }
}