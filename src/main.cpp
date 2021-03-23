// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// mLT                  motor         10              
// mLB                  motor         2               
// mRT                  motor         3               
// mRB                  motor         4               
// Controller1          controller                    
// leftIntake           motor         20              
// rightIntake          motor         5               
// mainRoll             motor         6               
// finalRoll            motor         11              
// rightEncoder         rotation      7               
// backEncoder          rotation      8               
// leftEncoder          rotation      9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "cmath"
#include "odom.h"
#include "custommath.h"

template <class U>
U logDrive(U s) {
  return pow(fabs((float)s), 2) / (sign(s)*100);
}

template <class T>
void debug(T arg) {
  Brain.Screen.print(arg);
  Brain.Screen.newLine();
}

void initDebug() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
}

using namespace vex;

// A global instance of competition
competition Competition;

void pre_auton(void) {
  vexcodeInit();

}

void autonomous(void) {
  thread ODOM = thread(tracking);
}

void usercontrol(void) {
  thread ODOM = thread(tracking);
  int RS = 0;
  int LS = 0;

  //debug(globalOrientation);
  while (1) {
    initDebug();
    debug(rX);
    debug(rY);
    debug(globalOrientation * 180 / M_PI);
    
    RS = logDrive(Controller1.Axis2.position(percent));
    LS = logDrive(Controller1.Axis3.position(percent));
    
    mLB.spin(forward, -LS, percent);
    mLT.spin(forward, LS, percent);
    mRT.spin(forward, -RS, percent);
    mRB.spin(forward, RS, percent);

    if (Controller1.ButtonR1.pressing()) {
      leftIntake.spin(forward, 100, percent);
      rightIntake.spin(reverse, 100, percent);
    } else if (Controller1.ButtonR2.pressing()) {
      leftIntake.spin(reverse, 100, percent);
      rightIntake.spin(forward, 100, percent);
    } else {
      leftIntake.stop(coast);
      rightIntake.stop(coast);
    }
    
    if (Controller1.ButtonL1.pressing()) {
      finalRoll.spin(forward, 100, percent);
      mainRoll.spin(forward, 100, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      finalRoll.spin(reverse, 100, percent);
      mainRoll.spin(reverse, 100, percent);
    } else {
      finalRoll.stop(hold);
      mainRoll.stop(hold);
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  rightEncoder.resetPosition();
  backEncoder.resetPosition();
  leftEncoder.resetPosition();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
