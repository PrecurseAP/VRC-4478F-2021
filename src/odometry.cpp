#include "odometry.h"
#include "math.h"

using namespace odom;

const float odom::wheelCirc = 2.75 * M_PI;
const float odom::chassisWidth = 1;//fsp[adjf[asdpijf[apsdjf[apisdjf[aisdwnrpbgpuiebjjjeklklwescdrtfvgybuhn please change later
const float odom::_2pi = M_PI * 2;
bool odom::kill = false;

odom::Position pose = {0.0, 0.0, 0.0};

float odom::angleWrap(float val) {
  while(val < -_2pi) {
    val += _2pi;
  }
  while(val >= _2pi) {
    val -= _2pi;
  }
  return val;
}

int odom::trackingLoop(Position* container) {
  using namespace vex;

  odom::kill = false;

  float prevL = 0;
  float prevR = 0;

  while(!kill) {
    float dX = 0.0;
    float dY = 0.0;
    float dTheta = 0.0;
    //initialize values

    float currL = twLeft.position(rev) * wheelCirc;
    float currR = twRight.position(rev) * wheelCirc;

    float dL = currL - prevL;
    float dR = currR - prevR;

    float centerArcLength = (dL + dR) / 2;

    dTheta = (dL - dR) / chassisWidth;

    float radius = (dTheta == 0) ? 0 : centerArcLength / dTheta;
    dX = (dTheta == 0) ? 0 : (radius - radius*cos(dTheta));
    dY = (dTheta == 0) ? centerArcLength : radius*sin(dTheta);

    prevL = currL;
    prevR = currR;

    container->x += (dX * cos(container->theta)) + (dY * sin(container->theta));
    container->y += (dY * cos(container->theta)) + (dX * sin(container->theta));
    container->theta = odom::angleWrap(container->theta + dTheta);
  }
  return 1;
}