//i like how the name of the file is a palindrome
#include <tuple>
#include "ppc.h"
#include "odometry.h"

using namespace ppc;

float ppc::lookahead = 18.0;
float ppc::lookaheadKF = .5;

void ppc::loop(pathing::Path* path) {
  bool done = false;
  while(!done) {
    pathing::Point closest;
    float dClosest;

    std::tie(closest, dClosest) = path->nearestPointAndDistance(pathing::Point(odom::pose.x, odom::pose.y));

    float newLookahead = lookahead - (dClosest * lookaheadKF);
    newLookahead = (newLookahead < 0) ? 1 : newLookahead;

    pathing::Point target = path->getPointAtIndex(closest.ind);

    float distanceToLookaheadPoint = sqrt(sq(target.x - odom::pose.x) + sq(target.y - odom::pose.y));

    float forwardPower = distanceToLookaheadPoint;

    float targetBearing = atan2(target.x - odom::pose.x, target.y - odom::pose.y);

    while (targetBearing < 0) {
      targetBearing += odom::_2pi;
    }

    float currentBearing = atan2(sin(odom::pose.theta), cos(odom::pose.theta));
  }
}