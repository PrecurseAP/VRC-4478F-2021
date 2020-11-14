#include "vex.h"
#include "robot-config.h"
#include "../src/odom/odom.h"
#include "../src/drive/drive.h"
#include "../src/pre-auton/pre-auton.h"
#include "../src/drive/drive.h"

thread ODOM;

using namespace vex;
//////
competition Competition;
void spinMotors(int fL, int fR, int bR, int bL) {
  frontLeft.spin(forward, fL, percent);
  frontRight.spin(forward, -fR, percent);
  backRight.spin(forward, -bR, percent);
  backLeft.spin(forward, bL, percent);
}
void pre_auton(void) {
  GYRO.calibrate();
  wait(3000, msec);
  renderScreen(); //draw the brain on the screen once.
  Brain.Screen.pressed(touchScreenLogic); //callback so that the drawing and logic code is only executed when the screen is touched. (this saves tons of resources as opposed to a loop)
}
 //////////
void autonomous(void) {
  //wait(3000, msec);
  
  leftFlipOut.spin(forward, 100, percent);
  rightFlipOut.spin(forward, 100, percent);
  wait(1000, msec);
  leftFlipOut.spin(reverse, 50, percent);
  rightFlipOut.spin(reverse, 50, percent);
  Brain.Screen.clearScreen(); //vamos no auton
  moveToPoint(25.8, 6.9, 180);
  wait(30, msec);
  resetPos();
  wait(30, msec);
  upperRollers.spin(reverse, 100, percent);
  bottomRollers.spin(forward, 100, percent);
  wait(2000, msec);

  upperRollers.stop(hold);
  bottomRollers.stop(hold);
  moveToPoint(42, -5.5, 45);
  wait(200, msec);
  resetPos();
  spinMotors(20, 20, 20, 20);
  wait(300, msec);
  stopAllDrive(hold);
  leftFlipOut.spin(forward, 100, percent);
  rightFlipOut.spin(forward, 100, percent);
  upperRollers.spin(reverse, 100, percent);
  bottomRollers.spin(forward, 100, percent);
  wait(2000, msec);
  bottomRollers.stop(hold);
  upperRollers.stop(hold);
  leftFlipOut.spin(reverse, 50, percent);
  rightFlipOut.spin(reverse, 50, percent);
  
}

void usercontrol(void) {
  /**
   * All of our drive code, used to move the robot around.
   */
  ODOM.thread::interrupt();
  
  _drive();
}

int main() {
  vexcodeInit();

  thread ODOM = thread(tracking);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) { wait(100, msec); }
} 