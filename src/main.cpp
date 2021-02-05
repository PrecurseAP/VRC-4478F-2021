#include "vex.h"
#include "robot-config.h"
#include "odom.h"
#include "drive.h"
#include "pre-auton.h"
#include "drive.h"

using namespace vex;

competition Competition;

void pre_auton(void) {
  renderScreen(); //draw the field on the screen once.
  Brain.Screen.pressed(touchScreenLogic); //callback so that the drawing and logic code is only executed when the screen is touched. (this saves tons of resources as opposed to a loop)
}

void autonomous(void) {
  thread ODOM = thread(tracking);

  resetPos();

  leftFlipOut.spin(forward, 100, percent);
  rightFlipOut.spin(forward, 100, percent);

  wait(800, msec);

  leftFlipOut.spin(reverse, 90, percent);
  rightFlipOut.spin(reverse, 90, percent);
  wait(400, msec);
  leftFlipOut.stop(hold);
  rightFlipOut.stop(hold);

  moveToPoint(23, 7.7, 180, 7, 8, 1);
  spinMotors(35, 35, 35, 35);
  wait(200, msec);
  stopAllDrive(hold);
  upperRollers.spin(forward, 100, percent);
  bottomRollers.spin(forward, 100, percent);
  wait(1000, msec);
  upperRollers.stop(hold);
  bottomRollers.stop(hold);
  
  moveToPoint(-21, 4, 226, 5, 5, 1.1);
  leftFlipOut.spin(forward, 100, percent);
  rightFlipOut.spin(forward, 100, percent);
  spinMotors(40, 40, 40, 40);
  wait(750, msec);
  stopAllDrive(hold);
  wait(200, msec);
  upperRollers.spin(forward, 100, percent);
  bottomRollers.spin(forward, 100, percent);
  leftFlipOut.spin(reverse, 60, percent);
  rightFlipOut.spin(reverse, 60, percent);
  wait(1300, msec);
  spinMotors(-100, -100, -100, -100);
  wait(600, msec);
  stopAllDrive(hold);
  upperRollers.stop();
  bottomRollers.stop();
  leftFlipOut.stop(hold);
  rightFlipOut.stop(hold);
  moveToPoint(72, 13, 135, 5, 3, .8);

  /*
  leftFlipOut.spin(forward, 100, percent);
  rightFlipOut.spin(forward, 100, percent);
  wait(1000, msec);
  leftFlipOut.spin(reverse, 50, percent);
  rightFlipOut.spin(reverse, 50, percent);
  Brain.Screen.clearScreen(); //vamos no auton
  swerve(25.8, 6.9, 180);
  wait(30, msec);
  resetPos();
  wait(30, msec);
  spinMotors(20, 20, 20, 20);
  wait(250, msec);
  stopAllDrive(hold);
  upperRollers.spin(reverse, 100, percent);
  bottomRollers.spin(forward, 100, percent);
  wait(2000, msec);
  
  upperRollers.stop(hold);
  bottomRollers.stop(hold);
  */
  /*swerve(42, -5.5, 45);
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
  rightFlipOut.spin(reverse, 50, percent);*/
}

void usercontrol(void) {
  ODOM.thread::interrupt();
  
  driveTheDamnRobot();
}

int main() {
  vexcodeInit();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  resetGyro();
  pre_auton();

  while (true) { wait(100, msec); }
} 