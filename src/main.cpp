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

  //leftFlipOut.spin(forward, 100, percent);
  //rightFlipOut.spin(forward, 100, percent);

  //wait(800, msec);

  //leftFlipOut.spin(reverse, 90, percent);
  //rightFlipOut.spin(reverse, 90, percent);
  //wait(400, msec);
  //leftFlipOut.stop(hold);
  //rightFlipOut.stop(hold);
  moveToPoint(23, 6.7, 180, 7, 6, 1);
  spinMotors(60, 60, 60, 60);
  wait(200, msec);
  stopAllDrive(hold);
  upperRollers.spin(forward, 100, percent);
  bottomRollers.spin(forward, 100, percent);
  wait(850, msec);
  upperRollers.stop(hold);
  bottomRollers.stop(hold);
  
  leftFlipOut.spin(forward, 100, percent);
  rightFlipOut.spin(forward, 100, percent);
  moveToPoint(-20, 5, 226, 4, 4, 1);
  //leftFlipOut.spin(forward, 100, percent);
  //rightFlipOut.spin(forward, 100, percent);
  spinMotors(27, 27, 27, 27);
  wait(700, msec);
  upperRollers.spin(forward, 100, percent);
  bottomRollers.spin(forward, 100, percent);  

  wait(300, msec);  
  leftFlipOut.stop(hold);
  rightFlipOut.stop(hold);
  stopAllDrive(hold);
  //leftFlipOut.spin(reverse, 60, percent);
  //rightFlipOut.spin(reverse, 60, percent);
  wait(1100, msec);
  spinMotors(-100, -100, -100, -100);
  wait(1100, msec);
  stopAllDrive(hold);
  upperRollers.stop();
  bottomRollers.stop();
  //leftFlipOut.stop(hold);
  //rightFlipOut.stop(hold);
  moveToPoint(63, 14, 134, 3, 3, 1);
  leftFlipOut.spin(forward, 100, percent);
  rightFlipOut.spin(forward, 100, percent);
  spinMotors(30, 30, 30, 30);
  wait(300, msec);
  upperRollers.spin(forward, 100, percent);
  bottomRollers.spin(forward, 100, percent);
  wait(1100, msec);
  stopAllDrive(hold);
  wait(200, msec);
  //upperRollers.spin(forward, 100, percent);
  //bottomRollers.spin(forward, 100, percent);
}

void usercontrol(void) {
  ODOM.thread::interrupt();
  
  driveTheDamnRobot();
}

int main() {
  vexcodeInit();
  
  resetGyro();
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) { wait(100, msec); }
} 