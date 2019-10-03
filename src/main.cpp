/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Masuk Robotics                                   */
/*    Created:      Mon Sep 23 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

//this next section lists all of our functions, the ones with 'driver' are for driver control, the ones with 'auton' are for the autonomus period

// this is the function we will use to drive in autonomus, the final 4 conditions determine the speed of each indivudal motor, so we can turn, drive straight, and strafe with one function
void driveAuton (int leftFrontRotations, int rightFrontRotations, int leftBackRotations, int rightBackRotations, int globalSpeed) {
  leftBack.setVelocity(globalSpeed, velocityUnits::pct);
  leftFront.setVelocity(globalSpeed, velocityUnits::pct);
  rightBack.setVelocity(globalSpeed, velocityUnits::pct);
  rightFront.setVelocity(globalSpeed, velocityUnits::pct);
  leftBack.rotateFor(leftBackRotations, rotationUnits::rev, false);
  leftFront.rotateFor(leftFrontRotations, rotationUnits::rev, false);
  rightBack.rotateFor(rightBackRotations, rotationUnits::rev, false);
  rightFront.rotateFor(rightFrontRotations, rotationUnits::rev, false);
}
//this function is for moving the arm in autonomus
void armAuton (int time, int speed, int rotations) {
  leftArm.setVelocity(speed, velocityUnits::pct );
  rightArm.setVelocity(speed, velocityUnits::pct );
  leftArm.rotateFor( rotations, rotationUnits::rev, false );
  rightArm.rotateFor( rotations, rotationUnits::rev, false );
}

//this function will draw a gray square at given coordinates.
void graySquares(int x, int y) {

  Brain.Screen.setPenColor("#9e9e9e");

  for(int i = 1; i < 7; i++) {
    for(int j = 1; j < 7; j++) {
      if ((j == 1) && (i == 1)) {
        Brain.Screen.drawRectangle(x, y, 35, 35, "#9e9e9e");
      } else {
        Brain.Screen.drawRectangle(j * 35 + j - 36, i * 35 + i - 36, 35, 35, "#9e9e9e");
      }
    }
  } 

  task::sleep(25);
}

//this function draws a red square at coordinates
void redSquare() {

}

//this draws a blue square are given coordinates
void blueSquare() {

}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  bool preautComplete = false;
  
  while(!preautComplete) {
    graySquares(0, 0);
  }
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

void autonomous( void ) {
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

void usercontrol( void ) {
  // User control code here, inside the loop
  int strafe = 50;
  int armPCT = 100;
  int clawPCT = 100;
  int clawExtendPCT = 50;

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo 
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to 
    // update your motors, etc.
    // ........................................................................
     

    
    /*
    this is the controler map for controler one
    axis 3 = left side drive
    axis 2 = right side
    left button = strafe left 
    right button = strafe right
    L1 bumper = raise arm
    L2 bumper = lower arm
    R1 bumper = open claw
    R2 bumper = close claw
    up button = claw extend
    down button = claw retract
    */
    // The following code drives the robot
    leftBack.spin(directionType::fwd, controller1.Axis3.position(), velocityUnits::pct);
    leftFront.spin(directionType::fwd, controller1.Axis3.position(), velocityUnits::pct);
    rightBack.spin(directionType::rev, controller1.Axis2.position(), velocityUnits::pct);
    rightFront.spin(directionType::rev, controller1.Axis2.position(), velocityUnits::pct);

    // the following code is for strafing
    if (controller1.ButtonLeft.pressing()) {
      leftBack.spin(directionType::fwd, strafe, velocityUnits::pct);
      leftFront.spin(directionType::rev, strafe, velocityUnits::pct);
      rightFront.spin(directionType::rev, strafe, velocityUnits::pct);
      rightBack.spin(directionType::fwd, strafe, velocityUnits::pct);
    }
    //this strafes the other way
    if (controller1.ButtonRight.pressing()) {
      leftBack.spin(directionType::rev, strafe, velocityUnits::pct);
      leftFront.spin(directionType::fwd, strafe, velocityUnits::pct);
      rightFront.spin(directionType::fwd, strafe, velocityUnits::pct);
      rightBack.spin(directionType::rev, strafe, velocityUnits::pct);
    }
    // the following code raises the arm
    if (controller1.ButtonL1.pressing()) {
      leftArm.stop(brakeType::coast);
      rightArm.stop(brakeType::coast);
      leftArm.spin(directionType::fwd, armPCT, velocityUnits::pct);
      rightArm.spin(directionType::fwd, armPCT, velocityUnits::pct);
    }
    // this is the reverse, lowering the arm
    else if (controller1.ButtonL2.pressing()) {
      leftArm.spin(directionType::rev, armPCT, velocityUnits::pct);
      rightArm.spin(directionType::rev, armPCT, velocityUnits::pct);
    }
    //this makes sure that when we arent moving the arm it is locked in place
    else {
      leftArm.stop(brakeType::hold);
      rightArm.stop(brakeType::hold);
    }

    // the following code opens the claw
    if (controller1.ButtonR1.pressing()) {
      clawMotor.spin(directionType::fwd, clawPCT, velocityUnits::pct);
    }
      
    // the following code closes the claw
    if (controller1.ButtonR2.pressing()) {
      clawMotor.spin(directionType::rev, clawPCT, velocityUnits::pct);
    }
    //this code will extend the claw
    if (controller1.ButtonUp.pressing()) {
      clawExtend.stop(brakeType::coast);
      clawExtend.spin(directionType::fwd, clawExtendPCT, velocityUnits::pct);
    }

    //this code will retract the claw
    else if (controller1.ButtonDown.pressing()) {
      clawExtend.stop(brakeType::coast);
      clawExtend.spin(directionType::rev, clawExtendPCT, velocityUnits::pct);
    }

    // this locks the arm extender in place when it is not being used
    else {
      clawExtend.stop(brakeType::hold);
    }
    task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
}