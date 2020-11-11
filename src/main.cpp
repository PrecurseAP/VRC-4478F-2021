#include "vex.h"
#include "robot-config.h"
#include "../src/odom/odom.h"
#include "../src/drive/drive.h"
#include "../src/pre-auton/pre-auton.h"
#include "../src/drive/drive.h"

using namespace vex;
//////
competition Competition;

void pre_auton(void) {
  renderScreen(); //draw the brain on the screen once.
  Brain.Screen.pressed(touchScreenLogic); //callback so that the drawing and logic code is only executed when the screen is touched. (this saves tons of resources as opposed to a loop)
}
 
void autonomous(void) {
  Brain.Screen.clearScreen(); //vamos no auton

}
template <class T>
void moveToPoint(T x, T y, int finalAngle = 0) {
  float kPx = 2.5, kPy = 2.5, kPt = .4;
  float kDx = 0, kDy = 0, kDt = 0;
  updatePositionVars();
  float xLast = x - pos.x;
  float yLast = y - pos.y;
  float tLast = finalAngle - pos.theta; 
  float motorSpeeds[4];

  bool complete = false;

  while(!complete) {
    updatePositionVars();

    /*Calculate the x component of our movement*/
    float xError = x - pos.x;
    float xDer = xError - xLast;
    float xLast = xError;
    float xComp = kPx*xError + kDx*xDer;
    /*End x component calculation              */

    /*Calculate the y component of our movement*/
    float yError = y - pos.y;
    float yDer = yError - yLast;
    float yLast = yError;
    float yComp = kPy*yError + kDy*yDer;
    /*End y component calculation              */

    /*Calculate turn component of our movement */
    float tError = finalAngle - pos.theta;
    float tDer = tError - tLast;
    float tLast = tError;
    float tComp = (kPt*tError) + (kDt*tDer); 
    /*End turn component calculation           */

    float targetTheta = datan2((x - pos.x), (y - pos.y)); 
    float tDist = pos.theta - targetTheta;

    xComp *= dcos(tDist);
    yComp *= dsin(tDist);

    motorSpeeds[0] = xComp - yComp + tComp;
    motorSpeeds[1] = xComp + yComp + tComp;
    motorSpeeds[2] = -xComp - yComp + tComp;
    motorSpeeds[3] = -xComp + yComp + tComp;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(pos.x);
    Brain.Screen.newLine();
    Brain.Screen.print(pos.y);
    Brain.Screen.newLine();
    Brain.Screen.print(pos.theta);
    Brain.Screen.newLine();
    Brain.Screen.print(motorSpeeds[3]);
    Brain.Screen.newLine();
    Brain.Screen.print(7);
    Brain.Screen.newLine();
    std::cout << xComp << std::endl;
    std::cout << yComp << std::endl;
    std::cout << tComp << std::endl;
    std::cout << std::endl;
    float maxValue = MAX(fabs(motorSpeeds[0]), fabs(motorSpeeds[1]), fabs(motorSpeeds[2]), fabs(motorSpeeds[3]));
    if (maxValue > 100) {
      for (int i = 0; i <= 3; i ++) {
        motorSpeeds[i] *= (100 / maxValue);
      }
    }
    frontLeft.spin(forward, motorSpeeds[0], percent);
    backLeft.spin(forward, motorSpeeds[1], percent);
    frontRight.spin(forward, motorSpeeds[2], percent);
    backRight.spin(forward, motorSpeeds[3], percent);    //spin the motors at their calculated speeds.
    

    if ((sqrt(xError*xError + yError*yError) < 1) && tError < 1.5) {
      complete = true;
      stopAllDrive(hold);
    }

    wait(10, msec);
  }
}
void usercontrol(void) {
  /**
   * All of our drive code, used to move the robot around.
   */
  wait(2500, msec);
  moveToPoint(25, 20, 180);
  //_drive();
}

int main() {
  vexcodeInit();

  thread ODOM = thread(tracking);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) { wait(100, msec); }
} 