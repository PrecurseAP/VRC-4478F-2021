
#include "vex.h"
#include "robot-config.h"
#include "../src/odom/odom.h"

#include "../src/pre-auton/pre-auton.h"
#include "../src/drive/drive.h"

using namespace vex;

competition Competition;

void pre_auton(void) {
  renderScreen(); //draw the brain on the screen once.
  Brain.Screen.pressed(touchScreenLogic); //callback so that the drawing and logic code is only executed when the screen is touched. (this saves tons of resources as opposed to a loop)
}
 
void autonomous(void) {
  Brain.Screen.clearScreen(); //vamos no auton

}

void usercontrol(void) {
  /**
   * All of our drive code, used to move the robot around.
   */
  wait(2500, msec);
  Controller1.ButtonA.pressed(resetGyro); //bind a button to reset the gyro

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