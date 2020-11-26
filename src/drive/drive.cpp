#include "../src/drive/drive.h"
#include "vex.h"
#include "../src/aidenmath/custommath.h"
#include "cmath"
#include "vex_timer.h"

//logarithmic drive functions. this means that lower joystick values are skewed towards zero while higher once rapidly scale up to max speed.
float logDrive_shallow(float cv) {
  //pretty slight logarithmic drive, lower joystick values are only slightly skewed towards zero.
  return (cv*cv) / (sign(cv)*127.0);
}
float logDrive_shallowish(float cv) {
  //less intense logarithmic drive, reaches high speeds at a good distance from joystick limits
  return pow(fabs(cv), 1.5) / (sign(cv)*sqrt(127.0));
}
float logDrive_steepish(float cv) {
  //pretty steep logarithmic drive, reaches max speed pretty close to the edge of the joystick limits
  return pow(fabs(cv), 2.5) / (sign(cv)*pow(127, 1.5));
}
float logDrive_steep(float cv) {
  //extremely steep logarithmic drive function, only reaches max speed at very high joystick values
  return pow(cv, 3) / 16129.0; //127^2
}

void stopAllDrive(brakeType bT) {
  /** 
   * Stops all drive motors with a specific brakeType.
   */
  backLeft.stop(bT);
  backRight.stop(bT);
  frontLeft.stop(bT);
  frontRight.stop(bT);
}

void spinMotors(int fL, int fR, int bR, int bL) {
  /**
   * Basic movement function to move all the drive motors at a certain speed. this is mainly used for imprecise forward, backward, left, and right movements.
   */
  frontLeft.spin(forward, fL, percent);
  frontRight.spin(forward, -fR, percent);
  backRight.spin(forward, -bR, percent);
  backLeft.spin(forward, bL, percent);
}

void resetGyro() {
  /**
   * Stops all drive motors then calibrates the gyro. I have to experiment with delays to make sure that the code only resumes when the gyro is done.
   */
  GYRO.calibrate();
  while(GYRO.isCalibrating()) {
    wait(100, msec);
  }
}

timer turningTimer;

enum motorIndex {
  bL = 0,
  bR = 1,
  fR = 2,                 //enum to simplify human readability for motor speed indices in the motor speeds array
  fL = 3                  //this is probably a bad and convoluted solution to a non-problem but aiden does what he does
};

float goalAngle = 0.01;
float angleIntegral = 0;
float angleError = 0; 
float previousError = 0; 
float turnValue = 0;
bool autoTurn = false;
float kP = 1.4;
float kI = 0.00003;
float kD = 0.4;
float mSpd[4];
float angleDerivative = 0;
float normalizer = 0;

void _drive() {
  while(1) {
    float gyroAngle = GYRO.heading(degrees);       //grab and store the gyro value

    int joyX = Controller1.Axis4.position();       // Set variables for each joystick axis
    int joyY = -Controller1.Axis3.position();      // joyY is negative because driving would be backwards otherwise
    float joyZ = Controller1.Axis1.position()/1.7;    // this here is the turning axis. It is divided by 1.7 to scale the joystick value down a little

    float vel = sqrt((joyX * joyX) + (joyY * joyY)); //get velocity value out of joystick values

    float x2 = vel * (dcos(datan2(joyY, joyX) - gyroAngle));     //i believe these generate coordinates based off the joystick
    float y2 = vel * (dsin(datan2(joyY, joyX) - gyroAngle));     //values. they are used to calculate the direction the robot should move by simulating a graph


    if (x2 == 0) 
      x2 = 0.00001; //safeguard against x2 being zero so no errors occur.

    float datanx2y2 = datan(y2/x2);            //save the code from calculating this every single run of the for loop
    float sqrtx2y2 = sqrt((x2*x2) + (y2*y2));  //^^ i believe this line finds the length of the segment made by the joystick to the home position
      
    for(int i = 0, addAngle = 45; i <= 3; i++, addAngle += 90)  //calculates motor speeds and puts them into an array
      mSpd[i] = -dsin(datanx2y2 + addAngle) * sqrtx2y2;  //"addAngle" offsets each speed calculation by 90 degrees to compensate for the wheel orientation in xdrive

    if (x2 < 0)                                         //Inverts the motors when x2 is less than 0 to account for the nonnegative sine curve
      for(int i = 0; i <= 3; i++)
        mSpd[i] *= -1;

    //BEGIN TURNING CODE***********************/
    if (fabs(joyZ) > 1) {
      autoTurn = false;
      turnValue = 0;
      turningTimer.reset();

      for(int i = 0; i <= 3; i++) {
        mSpd[i] += joyZ; //apply turning
      }
    } else if (turningTimer.time() >= 1000) {

      if (autoTurn == false) {
        autoTurn = true;
        goalAngle = GYRO.heading(degrees);                //switch mode to angle correction and setup for PID
      }

      previousError = angleError;                         //store previous error value for use in derivative

      angleError = gyroAngle - goalAngle;                 //difference between the current angle and the goal angle

      if (angleError > 180) {                             //adjust the angle error to force the pid to follow the shortest
        angleError = -((360 - gyroAngle) + goalAngle);    //direction to the goal
      }
      if (angleError < -180) {                            //^^second part of above comment
        angleError = (360 - goalAngle) + gyroAngle;       //
      }

      if (fabs(angleError) < 2) {                        //if the angle error is small enough, activate the integral
        angleIntegral += angleError;                      //
      } else {                                            //
        angleIntegral = 0;                                //set it to 0 if it's too big
      }                               

      angleDerivative = angleError - previousError;     //calculation of derivative pid value

      turnValue = (angleError*kP) + (kI*angleIntegral) + (kD*angleDerivative); //final pid calculation

      for(int i = 0; i <= 3; i++) {
        mSpd[i] -= turnValue; //apply turning
      }
    } 

    //END TURNING CODE**************************/

    //BEGIN NORMALIZER CODE*********************/
    float maxAxis = MAX(std::abs(joyX), std::abs(joyY), fabs(turnValue), fabs(joyZ)); //Find the maximum input given by the controller's axes and the angle corrector
    float maxOutput = MAX(fabs(mSpd[0]), fabs(mSpd[1]), fabs(mSpd[2]), fabs(mSpd[3])); //Find the maximum output that the drive program has calculated

    if (maxOutput == 0 || maxAxis == 0) {
      normalizer = 0; //Prevent the undefined value for normalizer
    } else {
      normalizer = maxAxis / maxOutput; //calculate normalizer
    }

    for (int i = 0; i <= 3; i++) {
      mSpd[i] *= normalizer; //caps motor speeds to 100 without losing the ratio between each value
    }
    /*float maxValue = MAX(fabs(mSpd[0]), fabs(mSpd[1]), fabs(mSpd[2]), fabs(mSpd[3]));
    if (maxValue > 100) {
      for (int i = 0; i <= 3; i ++) {
        mSpd[i] *= (100 / maxValue);
      }
    }*/
    //END NORMALIZER CODE***********************/

    backLeft.spin(forward, mSpd[bL], percent);
    backRight.spin(forward, mSpd[bR], percent);    //spin the motors at their calculated speeds.
    frontRight.spin(forward, mSpd[fR], percent);
    frontLeft.spin(forward, mSpd[fL], percent);

    if(Controller1.ButtonR1.pressing()) {     //brings ball straight up and into tower
      leftFlipOut.spin(forward, 200, rpm);
      rightFlipOut.spin(forward, 200, rpm); 
    } else {
      leftFlipOut.stop(hold);
      rightFlipOut.stop(hold);
    }
    if(Controller1.ButtonR2.pressing()) { //bring balls up, does not shoot them out
      leftFlipOut.spin(reverse, 100, rpm);
      rightFlipOut.spin(reverse, 100, rpm); 
    } else if (!Controller1.ButtonR1.pressing()) {
    leftFlipOut.stop(hold);
      rightFlipOut.stop(hold);
    }     
    if(Controller1.ButtonL1.pressing()) { //brings ball to hoarder cell, by spinning the roller above it backwards
      bottomRollers.spin(forward, 100, percent);
      upperRollers.spin(reverse, 100, percent);
    }
    if (Controller1.ButtonL2.pressing()) { 
      bottomRollers.spin(reverse, 100, percent);
      upperRollers.spin(forward, 100, percent);
    } else if (!Controller1.ButtonL1.pressing()) {
      bottomRollers.stop(hold);
      upperRollers.stop(hold);
    }
    //button b make intake retract
    wait(10, msec); 
  }
}
void driveNew() {
  /**
   * Brand new optimized and very WIP drive function designed to not be so cluttered and be a little faster, now with smoother control courtesy of logarithmic drive.
   */
  while(1) {
    float gyroAngle = GYRO.heading(degrees);       //grab and store the gyro value, it is the orientation of the bot

    float joyX = logDrive_shallowish(-Controller1.Axis4.position(percent));       // this is the left and right axis
    float joyY = logDrive_shallowish(Controller1.Axis3.position(percent));      // this is the forward and backward axis
    float joyZ = logDrive_shallowish(Controller1.Axis1.position(percent))/2;    // this here is the turning axis.

    float magnitude = sqrt((joyX*joyX) + (joyY*joyY)) / M_SQRT2; //this calculates the magnitude of the direction vector of the joystick. 
    float theta = atan2f(joyX, joyY) + (gyroAngle*(M_PI/180)); //this calculates the reference angle of the joystick coordinates offset by the gyro.

    float x2 = magnitude * cos(theta);  //these two lines generate new robot centric values based on the magnitude of speed and the offset in angle.
    float y2 = magnitude * sin(theta);

    mSpd[0] = x2 - y2 + joyZ;
    mSpd[1] = x2 + y2 + joyZ;
    mSpd[2] = -x2 - y2 + joyZ; //this is where the x, y, and turn components of holonomic movement are mushed together to calculate motor speeds.
    mSpd[3] = -x2 + y2 + joyZ;

    float maxAxis = MAX(fabs(joyX), fabs(joyY), fabs(joyZ)); //Find the maximum input given by the controller's axes and the angle corrector
    float maxOutput = MAX(fabs(mSpd[0]), fabs(mSpd[1]), fabs(mSpd[2]), fabs(mSpd[3])); //Find the maximum output that the drive program has calculated

    if (maxOutput == 0 || maxAxis == 0) {
      normalizer = 0; //Prevent the undefined value for normalizer
    } else {
      normalizer = maxAxis / maxOutput; //calculate normalizer
    }

    for (int i = 0; i <= 3; i++) {
      mSpd[i] *= normalizer; //caps motor speeds to the greatest input without losing the ratio between each speed, so as to not warp the direction of movement too much.
    }

    frontLeft.spin(forward, mSpd[0], percent);
    backLeft.spin(forward, mSpd[1], percent);
    frontRight.spin(forward, mSpd[2], percent);
    backRight.spin(forward, mSpd[3], percent);    //spin the motors at their calculated speeds.
    //bear in mind that this new driving function is in a testing state and there is no automatic angle correction or roller/intake movement. There is only translational movement.
   
    if(Controller1.ButtonR1.pressing()) {     //brings ball straight up and into tower
      leftFlipOut.spin(forward, 200, rpm);
      rightFlipOut.spin(forward, 200, rpm); 
    } else {
      leftFlipOut.stop(hold);
      rightFlipOut.stop(hold);
    }
    if(Controller1.ButtonR2.pressing()) { //bring balls up, does not shoot them out
      leftFlipOut.spin(reverse, 100, rpm);
      rightFlipOut.spin(reverse, 100, rpm); 
    } else if (!Controller1.ButtonR1.pressing()) {
    leftFlipOut.stop(hold);
      rightFlipOut.stop(hold);
    }     
    if(Controller1.ButtonL1.pressing()) { //brings ball to hoarder cell, by spinning the roller above it backwards
      bottomRollers.spin(forward, 100, percent);
      upperRollers.spin(reverse, 100, percent);
    }
    if (Controller1.ButtonL2.pressing()) { 
      bottomRollers.spin(reverse, 100, percent);
      upperRollers.spin(forward, 100, percent);
    } else if (!Controller1.ButtonL1.pressing()) {
      bottomRollers.stop(hold);
      upperRollers.stop(hold);
    }
    
    wait(10, msec); 
  }
}