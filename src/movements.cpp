/************************************
* File for movement functions, things that are done in autonomous, etc
************************************/

#include "movements.h"
#include "utils.h"
#include "vex.h"

bool clawState = false;

void clawToggle() {
  clawPiston.set(!clawState);
  clawState = !clawState;
}

void raiseLift(int val /* = 100*/, bool wait /* = true*/) {
  mArm.setVelocity(100, percent);
  mArm.spinToPosition(val, degrees, wait);
}

void lowerLift(bool wait /* = true*/) {
  mArm.spinToPosition(-40, degrees, wait);
}

void spinConveyor() {
  mConveyor.spin(forward, 169, rpm);
}

void lowerTilter(int speed/* = 100*/, int val /*= -540*/, bool wait /* = true*/) {
  mLTray.setVelocity(speed, percent);
  mRTray.setVelocity(speed, percent);
  mLTray.spinToPosition(val, degrees, false);
  mRTray.spinToPosition(val, degrees, wait);
}

int deploy(int val /* = 100*/) {
  raiseLift(true);
  lowerLift(false);

  return 0; 
}

void moveRightSide(int speed) {
  mRUpper.spin(forward, speed, percent);
  mRLower.spin(forward, speed, percent);
}

void moveLeftSide(int speed) {
  mLUpper.spin(forward, speed, percent);
  mLLower.spin(forward, speed, percent);
}

void stopAllDrive(brakeType bt) {
  mRUpper.stop(bt);
  mRLower.stop(bt);
  mLUpper.stop(bt);
  mLLower.stop(bt);
}

int turnToAngle(float goalAngle, int timeLimit, float kp/* = 4.0 */, float ki/* = .001 */, float kd/* = 3.0 */) {
  
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

    std::cout << error << std::endl;

    derivative = (error - prevError) / deltaTime;

    prevError = error;

    integral += error * deltaTime;

    float driveSpeed = kp*error + ki*integral + kd*derivative;
    
    mLUpper.spin(forward, driveSpeed, percent);
    mLLower.spin(forward, driveSpeed, percent);
    mRUpper.spin(forward, -driveSpeed, percent);
    mRLower.spin(forward, -driveSpeed, percent);

    if ((fabs(error) < .7) ||(Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }
    wait(20, msec);
  }

  return 1;
}

/*int moveStraight(float goalDistance, int timeLimit, float kp, float ki, float kd) {

  float startTime = Brain.timer(msec);
  float prevTime = startTime, deltaTime, currentTime;
  float integral = 0, error = 0, derivative = 0, prevError = 0;
  bool done = false;
  float currentLeft = (mLLower.position(degrees)/360.0) * 4.0 * M_PI;
  float currentRight = (mRLower.position(degrees)/360.0) * 4.0 * M_PI;

  Graph graph = Graph(currentAngle, 250);

  while(!done) {

    currentTime = Brain.timer(msec);
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;

    currentLeft = (mLLower.position(degrees)/360.0) * 4.0 * M_PI;
    currentRight = (mRLower.position(degrees)/360.0) * 4.0 * M_PI;

    graph.updateData(currentLeft);
    graph.drawGraph();

    LError = goalDistance - currentLeft;
    RError = goalDistance - currentRight;

    std::cout << LError << std::endl;

    LDerivative = (LError - LPrevError) / deltaTime;
    RDerivative = (RError - RPrevError) / deltaTime;

    LIntegral += LError * deltaTime;
    RIntegral += RError * deltaTime;

    float leftDriveSpeed = kp*LError + ki*LIntegral + kd*LDerivative;
    float rightDriveSpeed = kp*RError + ki*RIntegral + kd*RDerivative;

    mLUpper.spin(forward, leftDriveSpeed, percent);
    mLLower.spin(forward, leftDriveSpeed, percent);
    mRUpper.spin(forward, rightDriveSpeed, percent);
    mRLower.spin(forward, rightDriveSpeed, percent);

    if ((fabs(error) < .7) ||(Brain.timer(msec)-startTime > timeLimit)) {
      done = true;
      stopAllDrive(hold);
      break;
    }
    wait(20, msec);
  }

  return 1;
}
*/