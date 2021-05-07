#include "drive.h"
#include "vex.h"
#include "cmath"
#include "custommath.h"
#include <iostream>

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

using namespace vex;

// A global instance of competition
competition Competition;

void pre_auton(void) {
  vexcodeInit();
  //ahh oops
}

void turnToAngle(float theta, float constant = 3.0) {
  /*
  ** theta is the angle to which the robot will turn (absolute, 0 -> 360 degrees)
  ** constant is a value that increases or decreases the speed of the turn. larger values are unstable, smaller ones make it slooow
  */
  bool complete = false;

  while(!complete) { //control loop

    //wrap angle error around so that the robot will take the shortest path (not turning 330 degrees right to get to 30 degrees left)
    float angleError = theta - GYRO.heading(degrees);
    if (angleError > 180) {
      angleError = angleError - 360;
    } else if (angleError < -180) {
      angleError = 360 + angleError;
    }

    //calculate the universal motor speed
    //we use the square root of the error because that ensures that the robot slow down before it reaches its goal,
    //preventing it from overshooting and getting caught in a terrible loop
    float mspd = constant * sign(angleError) * sqrt(fabs(angleError));

    //spin the motors, mLB and mRB are reversed to compensate for motor gearing and orientation
    mLB.spin(forward, -mspd, percent);
    mLT.spin(forward, mspd, percent);
    mRT.spin(forward, mspd, percent);
    mRB.spin(forward, -mspd, percent);

    //end the control loop when we get near our goal. (error is tiny, .6 degrees works)
    if (fabs(angleError) < .6) {
      complete = true;
      stopAllDrive(hold);
      wait(200, msec);
    }
  }
}

const float wheelC = 2.75 * M_PI; //oooo spooky tracking wheel circumference

void moveStraight(float d, float c = 1.0) {
  /*
  ** d is the distance, in inches, to travel straight forward (or backward if negative)
  ** c is a constant that scales the speed of travel. higher means faster, lower means slower. default value is 1.
  */

  //reset encoder positions.
  leftEncoder.setPosition(0, degrees);
  rightEncoder.setPosition(0, degrees);
  
  //convert d into encoder rotations in degrees
  float d2 = (d / wheelC) * 360;

  float currLeft = 0;
  float currRight = 0;

  bool complete = false;

  while (!complete) { //control loop

    /* unused, could be used for future pid purposes
    float prevLeft = currLeft;
    float prevRight = currRight;
    */
    
    //store current encoder values
    currLeft = leftEncoder.position(degrees);
    currRight = rightEncoder.position(degrees);

    //calculate the "tilt" of the bot. This is a value representing how much the robot has changed its orientation.
    float tilt = currLeft - currRight;

    //calculate the distance to the goal, while factoring in the tilt so that the robot compensates.
    float rightError = d2 - currRight + tilt;
    float leftError = d2 - currLeft - tilt;

    //calculate the speeds at which each side of the robot should travel. c is a scalar and 12 is an arbitrary constant
    //if i dont divide the speed by some constant then the robot goes WAY too fast. (error is in degrees, gets to hundreds)
    float rightSpeed = c * rightError / 12;
    float leftSpeed = c * leftError / 12;

    initDebug();
    debug(rightError);
    debug(leftError);
    debug(tilt);
    debug("\n");

    //spin motors at their calculated speeds. Left side is scaled slightly.
    //the left scaling is annoying, our robot has a mechanical problem that makes it hard to drive straight over long distances
    //not enough time to fix it, we dont even know whats causing it. optimally i wouldnt need to scale the speeds like that.
    mLB.spin(forward, -leftSpeed*1.03, percent);
    mLT.spin(forward, leftSpeed*1.03, percent);
    mRT.spin(forward, -rightSpeed, percent);
    mRB.spin(forward, rightSpeed, percent);

    //end the control loop when we get close enough to our goal
    //i still need to tune these error values, we want as little error as possible while still getting to our goal fast enough.
    if ((tilt < 4) && ((fabs(rightError) + fabs(leftError))/2) < 9) {
      complete = true;
      stopAllDrive(hold);
      wait(200, msec);
    }
  }
}

void autonomous(void) { //when you dont comment any of your goddamn code
  /*GYRO.calibrate();
  wait(3000, msec);

  spinIntakes(100);
  moveStraight(47);
  stopIntakes(hold);
  turnToAngle(135);
  moveStraight(49);
  spinRollers(100);
  wait(1000, msec);
  stopRollers(hold);
  moveStraight(-43);
  turnToAngle(270);
  spinRollers(100);
  spinIntakes(100);
  wait(400, msec);
  stopRollers(hold);
  moveStraight(55);
  stopIntakes(hold);
  mainRoll.spin(forward, 10, percent);
  turnToAngle(180);
  stopRollers(hold);
  wait(300, msec);
  moveStraight(41.7);
  spinRollers(100);
  wait(1200, msec);
  stopRollers(hold);
  turnToAngle(270);
  spinIntakes(100);
  mainRoll.spin(forward, 10, percent);
  moveStraight(66);
  stopIntakes(hold);
  stopRollers(hold);
  turnToAngle(225);
  moveStraight(21);
  mainRoll.spin(forward, 75, percent);
  spinIntakes(100);
  wait(2500, msec);
  stopIntakes(hold);
  spinRollers(100);
  wait(750, msec);
  stopRollers(hold);*/
}

void usercontrol(void) {
  /*GYRO.calibrate();
  wait(2500, msec);
  //moveStraight(24, .7);
  turnToAngle(180);*/
  int RS = 0;
  int LS = 0;

  //idk if this is necessary i just want to make sure the motors are trying their hardest :)))
  mLB.setMaxTorque(100, percent);
  mRB.setMaxTorque(100, percent);
  mLT.setMaxTorque(100, percent);
  mRT.setMaxTorque(100, percent);

  while (1) { //drive FOREVER

    //logarithmic drive functions woooooo
    //this just gets the drive speeds, we have tank drive
    //i call it stank drive eheh
    RS = logDrive(Controller1.Axis2.position(percent));
    LS = logDrive(Controller1.Axis3.position(percent));
    
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
    wait(100, msec);
  }
}
//baheglo