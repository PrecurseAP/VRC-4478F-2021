//the shitty drive functions
#include "drive.h"
#include "vex.h"
#include "custommath.h"

const float wheelC = 2.75 * M_PI; //oooo spooky tracking wheel circumference


//stop all drive motors
void stopAllDrive(brakeType bT) {
  mLB.stop(bT);
  mLT.stop(bT);
  mRB.stop(bT);
  mRT.stop(bT); 
}
//spin the intakes
void spinIntakes(int s) {
  leftIntake.spin(forward, s, percent);
  rightIntake.spin(reverse, s, percent);
}
//stop the intakes
void stopIntakes(brakeType bt) {
  leftIntake.stop(bt);
  rightIntake.stop(bt);
}
//spin the rollers
void spinRollers(int s) {
  mainRoll.spin(forward, s, percent);
  finalRoll.spin(forward, s, percent);
}
//stop the rollers
void stopRollers(brakeType bt) {
  mainRoll.stop(bt);
  finalRoll.stop(bt);
}

void turnToAngle(float theta, float constant /*= 3.0*/) {
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
    if (fabs(angleError) < .75) {
      complete = true;
      stopAllDrive(hold);
      wait(200, msec);
    }
  }
}

void moveStraight(float d, float maxSpeed /*= 100.0*/, float c /*= 1.0*/) {
  /*
  ** d is the distance, in inches, to travel straight forward (or backward if negative)
  ** c is a constant that scales the speed of travel. higher means faster, lower means slower. default value is 1.
  ** maxSpeed is the maximum speed at which the robot is allowed to travel during the movement
  */

  //reset encoder positions.
  leftEncoder.setPosition(0, degrees);
  rightEncoder.setPosition(0, degrees);
  
  //convert d into encoder rotations in degrees
  float d2 = (fabs(d) / wheelC) * 360;

  float currLeft = 0;
  float currRight = 0;

  bool complete = false;

  while (!complete) { //control loop

    /* unused, could be used for future pid purposes
    float prevLeft = currLeft;
    float prevRight = currRight;
    */
    
    //store current encoder values
    currLeft = fabs(leftEncoder.position(degrees));
    currRight = fabs(rightEncoder.position(degrees));

    //calculate the "tilt" of the bot. This is a value representing how much the robot has changed its orientation.
    float tilt = currLeft - currRight;

    //calculate the distance to the goal, while factoring in the tilt so that the robot compensates.
    float rightError = d2 - currRight + tilt;
    float leftError = d2 - currLeft - tilt;

    //calculate the speeds at which each side of the robot should travel. c is a scalar and 12 is an arbitrary constant
    //if i dont divide the speed by some constant then the robot goes WAY too fast. (error is in degrees, gets to hundreds)
    float rightSpeed = c * rightError / 8;
    float leftSpeed = c * leftError / 8;

    //caps the maximum speed of the motors
    float max_ = MAX(fabs(rightSpeed), fabs(leftSpeed));
    if (max_ > maxSpeed) {
      rightSpeed *= (maxSpeed / max_);
      leftSpeed *= (maxSpeed / max_);
    }
    
    leftSpeed *= sign(d);
    rightSpeed *= sign(d);

    //spin motors at their calculated speeds.
    mLB.spin(forward, -leftSpeed, percent);
    mLT.spin(forward, leftSpeed, percent);
    mRT.spin(forward, -rightSpeed, percent);
    mRB.spin(forward, rightSpeed, percent);

    //end the control loop when we get close enough to our goal
    //i still need to tune these error values, we want as little error as possible while still getting to our goal fast enough.
    if ((tilt < 10) && ((fabs(rightError) + fabs(leftError))/2) < 18) {
      complete = true;
      stopAllDrive(hold);
      wait(200, msec);
    }
  }
}
