/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFront            motor         19              
// rightFront           motor         9               
// leftArm              motor         5               
// rightArm             motor         2               
// leftBack             motor         20              
// intakeLeft           motor         7               
// intakeRight          motor         8               
// rightBack            motor         10              
// autonSel             pot           B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;
int pathChoice = 0;

float floatAbs(float a) {
  if (a < 0) return a*-1; else return a;
}
void stopDrive() {
  leftFront.stop();
  rightFront.stop();
  leftBack.stop();
  rightBack.stop();
}
void drivePID(float target, float kP, float kI, float kD, float maxSpeed, bool wait, bool fwd) {
  Brain.Screen.setCursor(1,1);
  leftBack.setPosition(0, degrees);
  leftFront.setPosition(0, degrees);
  rightBack.setPosition(0, degrees);
  rightFront.setPosition(0, degrees);
  float deriv, error, prevError;
  float integ = 0;
  float intThres = 20;
  error = target;
  bool complete = false;
  directionType rightDir, leftDir;
  if (fwd == true) {
    leftDir = forward;
    rightDir = reverse;
  } else {
    leftDir = reverse;
    rightDir = forward;
  }
  //everything below here goes into a 
  while(!complete) {

    float leftVal = (floatAbs(leftFront.rotation(degrees)) + floatAbs(leftBack.rotation(degrees))) / 2;
    float rightVal = (floatAbs(rightFront.rotation(degrees)) + floatAbs(rightBack.rotation(degrees))) / 2;
    float currentVal = (leftVal + rightVal) / 2;

    prevError = error;

    error = target - currentVal; 

    if (floatAbs(error) < intThres) {
      integ += error;
    } else {
      integ = 0;
    }

    deriv = error - prevError;

    Brain.Screen.newLine();

    float SPEED = kP*error + kI*integ + kD*deriv;

    Brain.Screen.print(SPEED);

    SPEED = !(SPEED < maxSpeed) ? maxSpeed : SPEED;

    leftFront.spin(leftDir, SPEED, velocityUnits::pct);
    leftBack.spin(leftDir, SPEED, velocityUnits::pct);
    rightFront.spin(rightDir, SPEED, velocityUnits::pct);
    rightBack.spin(rightDir, SPEED, velocityUnits::pct);

    if (SPEED <= 9) {
      stopDrive();
      complete = true;
    }

    task::sleep(40);
  }
}

void turnPID(float target, float kP, float kI, float kD, bool wait, int dir) {
  //placeholder
}

void driveAuton (int leftFrontRotations, int rightFrontRotations, int globalSpeed) {
  leftFront.setVelocity(globalSpeed, velocityUnits::pct);
  rightFront.setVelocity(globalSpeed, velocityUnits::pct);
  leftFront.rotateFor(leftFrontRotations, rotationUnits::deg, false);
  rightFront.rotateFor(rightFrontRotations, rotationUnits::deg, false);
  leftBack.setVelocity(globalSpeed, velocityUnits::pct);
  rightBack.setVelocity(globalSpeed, velocityUnits::pct);
  leftBack.rotateFor(leftFrontRotations, rotationUnits::deg, false);
  rightBack.rotateFor(rightFrontRotations, rotationUnits::deg, false);
}
//this function is for moving the arm in autonomus
void armAuton (int speed, int degs) {
  leftArm.setVelocity(speed, velocityUnits::pct );
  rightArm.setVelocity(speed, velocityUnits::pct );
  leftArm.rotateFor( degs, rotationUnits::deg, false );
  rightArm.rotateFor( degs, rotationUnits::deg, false );
}
void intakeAuton(directionType dir) {
  intakeLeft.spin(dir, 100, velocityUnits::pct);
  intakeRight.spin(dir, 100, velocityUnits::pct);
}

int autonSelection;
void pre_auton( void ) {
  vexcodeInit();
  /*while(true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(autonSel.angle(degrees));
    Brain.Screen.newLine();
    int a = autonSel.angle(degrees);
    if ((a >= 0) && (a <= 15)) {
      Brain.Screen.print("Top Red");
      autonSelection = 1;
    } else if (a <= 42) {
      Brain.Screen.print("Bottom Red");
      autonSelection = 2;
    } else if (a <= 68) {
      Brain.Screen.print("Top Blue");
      autonSelection = 3;
    } else if (a <= 101) {
      Brain.Screen.print("Bottom Blue");
      autonSelection = 4;
    } else if (a <= 134) {
      Brain.Screen.print("Programming Skills");
      autonSelection = 5;
    } else {
      Brain.Screen.print("No program set");
      autonSelection = 0;
    }
    Brain.Screen.newLine();
    Brain.Screen.print(leftFront.rotation(degrees));
    Brain.Screen.newLine();
    Brain.Screen.print(leftBack.rotation(degrees));
    Brain.Screen.newLine();
    Brain.Screen.print(rightBack.rotation(degrees));
    Brain.Screen.newLine();
    Brain.Screen.print(rightFront.rotation(degrees));
    Brain.Screen.newLine();
    wait(50, msec);
  }*/
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  leftFront.setPosition(0, degrees);
  leftBack.setPosition(0, degrees);
  rightFront.setPosition(0, degrees);
  rightBack.setPosition(0, degrees);

  switch(autonSelection) {
    case 1: {
      drivePID(1000, 0.2, 0.1, 0.1, 70, true, true);
    }
    case 2: {

    }
    case 3: {

    }
    case 4: {

    }
    case 5: {

    }
  }
  
  }

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
double driveFactor = 3.00;

void switchDriveSpeed() {
  if (driveFactor == 1.25) {
    driveFactor = 3.00;
  } else if (driveFactor == 3.00) {
    driveFactor = 1.25;
  } else {
    driveFactor = 2; //backup in case something dumb happens and this code is reached
  }
}
void testPID() {
  drivePID(1000, 0.3, 0.2, 0.1, 80, true, true);
}
void usercontrol(void) {
  // User control code here, inside the loop
  int armPCT = 100;
  int intakePCT = 100;
  while (1) {
    Controller1.ButtonX.pressed(switchDriveSpeed); // <-- Changes the scale at which the drive motors spin. Useful for precision movements
    // The following code drives the robot, while accounting for DRIFT THAT SHOULDNT BE THERE
    if (abs(Controller1.Axis3.position()) >= 4) {
      leftFront.spin(directionType::fwd, Controller1.Axis3.position()/driveFactor, velocityUnits::pct);
      leftBack.spin(directionType::fwd, Controller1.Axis3.position()/driveFactor, velocityUnits::pct);
    } else {
      leftFront.stop();
      leftBack.stop();
    }

    if (abs(Controller1.Axis2.position()) >= 4) {
      rightFront.spin(directionType::rev, Controller1.Axis2.position()/driveFactor, velocityUnits::pct);
      rightBack.spin(directionType::rev, Controller1.Axis2.position()/driveFactor, velocityUnits::pct);
    } else {
      rightFront.stop();
      rightBack.stop();
    } 
    if (Controller1.ButtonUp.pressing()) {
      leftFront.spin(directionType::fwd, 50, velocityUnits::pct);
      leftBack.spin(directionType::fwd, 50, velocityUnits::pct);
      rightFront.spin(directionType::rev, 50, velocityUnits::pct);
      rightBack.spin(directionType::rev, 50, velocityUnits::pct);
    } else if (Controller1.ButtonDown.pressing()) {
      leftFront.spin(directionType::rev, 50, velocityUnits::pct);
      leftBack.spin(directionType::rev, 50, velocityUnits::pct);
      rightFront.spin(directionType::fwd, 50, velocityUnits::pct);
      rightBack.spin(directionType::fwd, 50, velocityUnits::pct);
    }
    // the following code raises the arm
    if (Controller1.ButtonL1.pressing()) {
      rightArm.spin(directionType::fwd, armPCT, velocityUnits::pct);
      leftArm.spin(directionType::fwd, armPCT, velocityUnits::pct);
    }
    // this is the reverse, lowering the arm
    else if (Controller1.ButtonL2.pressing()) {
      leftArm.spin(directionType::rev, armPCT-10, velocityUnits::pct);
      rightArm.spin(directionType::rev, armPCT-10, velocityUnits::pct);
    }
    //this makes sure that when we arent moving the arm it is locked in place
    else if (!Controller1.ButtonB.pressing()) {
      leftArm.stop(brakeType::hold);
      rightArm.stop(brakeType::hold);
    }

    // the following code opens the claw
    if (Controller1.ButtonR1.pressing()) {
      intakeLeft.spin(directionType::rev, intakePCT, velocityUnits::pct);
      intakeRight.spin(directionType::rev, intakePCT, velocityUnits::pct);
    }
      
    // the following code closes the claw
    else if (Controller1.ButtonR2.pressing()) {
      intakeLeft.spin(directionType::fwd, intakePCT, velocityUnits::pct);
      intakeRight.spin(directionType::fwd, intakePCT, velocityUnits::pct);
    }
    //make sure the motor stops when we arent pressing the buttons
    else {
      intakeLeft.stop(brakeType::hold);
      intakeRight.stop(brakeType::hold);
    }
    if (Controller1.ButtonLeft.pressing()) {
      leftArm.spin(directionType::fwd, 50, velocityUnits::pct);
    } else if (Controller1.ButtonRight.pressing()) {
      rightArm.spin(directionType::fwd, 50, velocityUnits::pct);
    } else if (Controller1.ButtonB.pressing()) {
      rightArm.rotateTo(leftArm.rotation(rotationUnits::rev), rotationUnits::rev, false);
    }
    task::sleep(50); //Sleep the task for a short amount of time to prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
