#include "drive.h"
#include "vex.h"
#include "cmath"
#include "custommath.h"
#include <iostream>

using namespace vex;

//LOOK AT ME I CAN USE TEMPLATES
//debug function prints crap to the screen
template <class T>
void debug(T arg) {
  Brain.Screen.print(arg);
  Brain.Screen.newLine();
}

//clear the brain and set cursor, usually runs in a loop along with debug();
void initDebug() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
}

//logarithmic drive function, scales the drive speed so that we may have more control at slower speeds
template <class U>
U logDrive(U s) {
  return (s*s) / (sign(s)*100);
}

//fuckin drive around without any tracking
void driveForwardNoPID(int s) {
  mLB.spin(forward, -s, percent);
  mLT.spin(forward, s, percent);
  mRT.spin(forward, -s, percent);
  mRB.spin(forward, s, percent);
}

// A global instance of competition
competition Competition;

void pre_auton(void) {
  vexcodeInit();
  //ahh oops
  //idk if this is necessary i just want to make sure the motors are trying their hardest :)))
  mLB.setMaxTorque(100, percent);
  mRB.setMaxTorque(100, percent);
  mLT.setMaxTorque(100, percent);
  mRT.setMaxTorque(100, percent);
}

void autonomous(void) { //when you dont comment any of your goddamn code BUT I DID
  GYRO.calibrate(); //calibrate gyro first (we could put this in pre-auton)
  wait(3000, msec);
  
  //first action; move forward and grab the ball in front of us
  spinIntakes(100);
  moveStraight(26.5, 70);
  stopIntakes(hold);

  //second action; turn towards corner toward, travel to it, shoot a ball into it
  turnToAngle(135);
  moveStraight(29, 60, .9);
  spinRollers(100);
  wait(1000, msec);
  stopRollers(hold);

  //third action; move backwards, turn to face the left side of the field
  moveStraight(-42, 50);
  turnToAngle(270);

  //fourth acton; drive forward while sucking in the ball in front of us, then stop in front of middle home tower
  spinRollers(100);
  spinIntakes(100);
  wait(400, msec);
  stopRollers(hold);
  moveStraight(25.5, 50, .75);
  stopIntakes(hold);

  //fifth action; turn toward the home tower, move to it, deposit a ball into it
  mainRoll.spin(forward, 10, percent);
  turnToAngle(180);
  stopRollers(hold);
  wait(400, msec);
  moveStraight(27.5, 70);
  spinRollers(100);
  wait(700, msec);
  stopRollers(hold);

  //sixth action; move backward, turn to the left again, then grab the ball sitting against the left wall
  moveStraight(-13, 40);
  turnToAngle(269);
  spinIntakes(100);
  mainRoll.spin(forward, 5, percent);
  moveStraight(58.5, 55);
  wait(100, msec);
  stopIntakes(hold);
  mainRoll.stop(hold);

  //seventh action; move backward, turn to face home corner tower, move to it, deposit ball
  moveStraight(-22.5, 55);
  turnToAngle(225);
  moveStraight(31, 55);
  spinRollers(100);
  wait(750, msec);
  stopRollers(hold);

  //eigth action; travel backwards, turn to middle ball, go forward to grab it
  moveStraight(-10, 40);
  turnToAngle(0);
  spinIntakes(90);
  moveStraight(47.3, 55);
  stopIntakes(hold);
  
  //ninth action; turn to face toward, move forward, deposit, then move backwards
  turnToAngle(270);
  moveStraight(6, 35);
  spinRollers(100);
  wait(900, msec);
  stopRollers(hold);
  moveStraight(-15, 50);

  //tenth action; turn to face far ball, move to grab, turn to tower
  turnToAngle(0);
  spinIntakes(90);
  moveStraight(48, 60);
  stopIntakes(hold);
  turnToAngle(303);
  
  //eleventh action; drive into far corner tower, deposit ball, back up, turn to face backwards
  moveStraight(20, 45);
  spinRollers(100);
  wait(900, msec);
  stopRollers(hold);
  moveStraight(-12, 55);
  turnToAngle(180);
  spinIntakes(100);
  spinRollers(100);
  wait(250, msec);
  stopIntakes(hold);
  stopRollers(hold);

  //twelvth action; move towards center tower, descore a ball, then score the ball!
  moveStraight(47, 65);
  turnToAngle(90);
  spinIntakes(-100);
  moveStraight(39.5, 70);
  driveForwardNoPID(60);
  stopAllDrive(hold);
  stopIntakes(hold);
  spinRollers(100);
  wait(1200, msec);
  stopRollers(hold);
}

void usercontrol(void) {
  int RS = 0, LS = 0;

  while (1) { //drive FOREVER

    //logarithmic drive functions woooooo
    //this just gets the drive speeds, we have tank drive
    //i call it stank drive eheh
    RS = Controller1.Axis2.position(percent);
    LS = Controller1.Axis3.position(percent);
    
    //go robot, go!! go shoot balls i guess
    mLB.spin(forward, -LS, percent);
    mLT.spin(forward, LS, percent);
    mRT.spin(forward, -RS, percent);
    mRB.spin(forward, RS, percent);

    //right buttons on controller spin the intakes in and out
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
    
    //the left buttons on the controller spin the rollers in and out
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
  leftEncoder.resetPosition();
  //can i bring our old robot to prom
  //you know, the x-drive one
  //my baby
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);//gamingk
  }
}
//baheglo