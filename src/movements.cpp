/************************************
* File for movement functions, things that are done in autonomous, etc
************************************/

#include "movements.h"
#include "utils.h"
#include "vex.h"
#include "odometry.h"

float driveRatio = 3.0/7.0;

void raiseLift(int val /* = 60*/, bool wait /* = true*/) {
  mLift.setVelocity(100, percent);
  mLift.spinToPosition(val, degrees, wait);
}

void lowerLift() {
  mLift.setVelocity(100, percent);
  mLift.spinToPosition(0, degrees, false);
}

void stopAllDrive(brakeType bt) {
  mFrontRight.stop(bt);
  mMidRight.stop(bt);
  mBackRight.stop(bt);
  mFrontLeft.stop(bt);
  mMidLeft.stop(bt);
  mBackLeft.stop(bt);
}

void basicDrive(int speed) {
  mFrontLeft.spin(forward, speed, percent);
  mMidLeft.spin(forward, speed, percent);
  mBackLeft.spin(forward, speed, percent);
  mFrontRight.spin(forward, speed, percent);
  mMidRight.spin(forward, speed, percent);
  mBackRight.spin(forward, speed, percent);
}

int deploy() {
  mLift.setVelocity(100, percent);
  mLift.spin(forward, 100, percent);
  wait(350, msec);
  mLift.spinToPosition(0, degrees, true);

  return 0;
}

void spinConveyor() {
  mConveyor.spin(forward, 400, rpm);
}

int turnToAngle(float goalAngle, float timeLimit, float kp/* = 4.0 */, float ki/* = .001 */, float kd/* = 3.0 */) {
  
  float startTime = Brain.timer(msec);
  float prevTime = startTime, deltaTime, currentTime;
  float integral = 0, error = 0, derivative = 0, prevError = 0;
  bool done = false;
  float currentAngle = GYRO.heading(degrees);

 Graph graph = Graph(currentAngle, 250, goalAngle);

  while(!done) {

    currentTime = Brain.timer(msec);
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;

    currentAngle = GYRO.heading(degrees);

    graph.updateData(currentAngle);
    graph.drawGraph();

    error = goalAngle - currentAngle;

    if (error > 180) {
      error = error - 360;
    } else if (error < -180) {
      error = 360 + error;
    }

    derivative = (error - prevError) / deltaTime;

    prevError = error;

    integral += error * deltaTime;

    float driveSpeed = kp*error + ki*integral + kd*derivative;
    
    mFrontLeft.spin(forward, driveSpeed, percent);
    mMidLeft.spin(forward, driveSpeed, percent);
    mBackLeft.spin(forward, driveSpeed, percent);
    mFrontRight.spin(forward, -driveSpeed, percent);
    mMidRight.spin(forward, -driveSpeed, percent);
    mBackRight.spin(forward, -driveSpeed, percent);

    if ((fabs(error) < .75) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      std::cout << "turn: " << Brain.timer(msec)-startTime << std::endl;
      break;
    }
    wait(20, msec);
  }

  return 1;
}

int turnWithTilterGoal(float d, float t, float kp, float ki, float kd) {
  return turnToAngle(d, t, kp, ki, kd);
}

int turnWith2Goals(float d, float t, float kp, float ki, float kd) {
  return turnToAngle(d, t, kp, ki, kd);
}

int turnWithClawGoal(float d, float t, float kp, float ki, float kd) {
  return turnToAngle(d, t, kp, ki, kd);
}

int moveStraight(float goalDistance, float timeLimit, float kp, float maxSpeed, float ki, float kd) {

  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mBackLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);

  float startTime = Brain.timer(msec);
  float prevTime = startTime, deltaTime, currentTime;
  float LIntegral = 0, RIntegral = 0, LError = 0, RError = 0, LDerivative = 0, RDerivative = 0, LPrevError = 0, RPrevError = 0;
  bool done = false;
  float currentLeft = (((mFrontLeft.position(degrees) / 360.0) * driveRatio) * 4.0 * 3.14);
  float currentRight = (((mFrontRight.position(degrees) / 360.0) * driveRatio) * 4.0 * 3.14);

  Graph graph = Graph(currentLeft, 250, goalDistance);

  while(!done) {

    currentTime = Brain.timer(msec);
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;

    currentLeft = (((mFrontLeft.position(degrees) / 360.0) * driveRatio) * 4.0 * 3.14);
    currentRight = (((mFrontRight.position(degrees) / 360.0) * driveRatio) * 4.0 * 3.14);

    graph.updateData(currentLeft);
    graph.drawGraph();

    LError = goalDistance - currentLeft;
    RError = goalDistance - currentRight;

    //std::cout << LError << std::endl;

    LDerivative = (LError - LPrevError) / deltaTime;
    RDerivative = (RError - RPrevError) / deltaTime;

    LPrevError = LError;
    RPrevError = RError;

    if (fabs(LError) < 12) {
      LIntegral += LError * deltaTime;
      RIntegral += RError * deltaTime;
    } else {
      LIntegral = 0;
      RIntegral = 0;
    }

    float leftDriveSpeed = kp*LError + ki*LIntegral + kd*LDerivative;
    float rightDriveSpeed = kp*RError + ki*RIntegral + kd*RDerivative;

    if (maxSpeed != 100) {
      if (fabs(leftDriveSpeed) > maxSpeed) {
        leftDriveSpeed = maxSpeed * (int(0) < leftDriveSpeed) - (leftDriveSpeed < int(0));
      }
      if (fabs(rightDriveSpeed) > maxSpeed) {
        rightDriveSpeed = maxSpeed * (int(0) < rightDriveSpeed) - (rightDriveSpeed < int(0));
      }
    }
    //std::cout << leftDriveSpeed << std::endl;
    mFrontLeft.spin(forward, leftDriveSpeed, percent);
    mMidLeft.spin(forward, leftDriveSpeed, percent);
    mBackLeft.spin(forward, leftDriveSpeed, percent);
    mFrontRight.spin(forward, rightDriveSpeed, percent);
    mMidRight.spin(forward, rightDriveSpeed, percent);
    mBackRight.spin(forward, rightDriveSpeed, percent);
    
    if (((fabs(LError) < .6) && (fabs(RError) < .6)) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      std::cout << "move: " << Brain.timer(msec) - startTime << std::endl;
      break;
    }
    wait(20, msec);
  }

  return 1;
}

void odomTurn(float gx, float gy, float t) {
  //float currentTheta = 450 - GYRO.heading(degrees);
  float rx = pose.x;
  float ry = pose.y;

  float dx = gx - rx;
  float dy = gy - ry;

  dx = (dx == 0) ? .01 : dx;
  dy = (dy == 0) ? .01 : dy;

  float gTheta = 0;

  if (dx < 0) {
    gTheta = (180/M_PI) * atan(dy/dx) + 180;
  } else {
    gTheta = (180/M_PI) * atan(dy/dx);
  }

  gTheta = fmod(fabs(gTheta-450), 360);

  if (gTheta < 0) {
    gTheta += 360;
  }
  if (gTheta > 360) {
    gTheta -= 360;
  }
  std::cout << gTheta << std::endl;
  turnWith2Goals(gTheta, t);
}

void odomStraight(float gx, float gy, float t) {

  float gTheta = GYRO.heading(degrees);

  float cx = pose.x;
  float cy = pose.y;

  float dx = gx - cx;
  float dy = gy - cy;

  float d = sqrt( (dx*dx) + (dy*dy) );

  moveStraight(d, t);
}

template <typename G> 
int sgn3(G val) { //SIGNUM
    return (G(0) < val) - (val < G(0));
}

void followArc(float r, float d, float t, float c2, int dir, bool bypass) {

  float startTime = Brain.timer(msec);

  float curvature = 1.0 / r;

  float prevX = pose.y;
  float prevY = pose.y;

  float dTotal = 0;

  bool done = false;

  while(!done) {
    float rX = pose.x;
    float rY = pose.y;

    float dX = rX - prevX;
    float dY = rY - prevY;

    prevX = rX;
    prevY = rY;

    dTotal += sqrt( dX*dX + dY*dY );

    float c = c2 / (2 + curvature*14.25);

    float v = c * std::cbrt(((d - (dTotal / 2)) / d));
    
    v = bypass ? c : v;

    float leftV = v * ((2 + dir*curvature*14.25) / 2);
    float rightV = v * ((2 - dir*curvature*14.25) / 2);

    /*if ((fabs(leftV) > 100) && (curvature > 0)) {
      rightV = 100 - (leftV - 100);
      leftV = 100;
    }
    if ((fabs(rightV) > 100) && (curvature < 0)) {
      leftV = 100 - (rightV - 100);
      rightV = 100;
    }*/

    mFrontLeft.spin(forward, leftV, percent);
    mMidLeft.spin(forward, leftV, percent);
    mBackLeft.spin(forward, leftV, percent);
    mFrontRight.spin(forward, rightV, percent);
    mMidRight.spin(forward, rightV, percent);
    mBackRight.spin(forward, rightV, percent);

    std::cout << leftV << std::endl;

    if ((fabs(d - dTotal) < 1) || ((Brain.timer(msec) - startTime) > t)) {
      done = true;
      stopAllDrive(hold);
    }
    wait(20, msec);
  }
}