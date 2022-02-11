/************************************
* File for movement functions, things that are done in autonomous, etc
************************************/

#include "movements.h"
#include "utils.h"
#include "vex.h"

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

 Graph graph = Graph(currentAngle, 250);

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

    if ((fabs(error) < .85) || (Brain.timer(msec)-startTime > timeLimit)) {
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

int moveStraight(float goalDistance, float timeLimit, float kp, float ki, float kd) {

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

  Graph graph = Graph(currentLeft, 250);

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
    //std::cout << leftDriveSpeed << std::endl;
    mFrontLeft.spin(forward, leftDriveSpeed, percent);
    mMidLeft.spin(forward, leftDriveSpeed, percent);
    mBackLeft.spin(forward, leftDriveSpeed, percent);
    mFrontRight.spin(forward, rightDriveSpeed, percent);
    mMidRight.spin(forward, rightDriveSpeed, percent);
    mBackRight.spin(forward, rightDriveSpeed, percent);
    
    if (((fabs(LError) < .7) && (fabs(RError) < .7)) || (Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      std::cout << "move: " << Brain.timer(msec) - startTime << std::endl;
      break;
    }
    wait(20, msec);
  }

  return 1;
}
