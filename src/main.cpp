#include "vex.h"
#include "trig.h"
#include "custommath.h"

#define _bL_ 0
#define _bR_ 1
#define _fR_ 2
#define _fL_ 3

using namespace vex;

competition Competition;

void stopAllDrive() {
  /** 
  *Stops all motors
  */
  backLeft.stop();
  backRight.stop();
  frontLeft.stop();
  frontRight.stop();
}

void resetGyro() {
  /**
  *Stops all motors then calibrates the gyro
  */
  stopAllDrive();
  GYRO.calibrate();
}
void pre_auton(void) {

  vexcodeInit();
}

void autonomous(void) {

}

void usercontrol(void) {
  resetGyro(); //calibrate the gyro and wait for it to complete
  wait(2000, msec);
  Controller1.ButtonA.pressed(resetGyro); //bind a button to reset the gyro

  double goalAngle = 0, angleIntegral = 0, angleError = 0;
  double normalizer, angleDerivative, previousAngle;
  double kP = 1.5, kI = 0.02, kD = 0.8;
  double motorSpeeds[4];
  while (1) {
    double gyroAngle = GYRO.heading();       //grab and store the gyro value

    double joyX = Controller1.Axis4.position();       // Set variables for each joystick axis
    double joyY = -Controller1.Axis3.position();      // joyY is negative because driving would be backwards otherwise
    double joyZ = Controller1.Axis1.position()/30.0;    // this here is divided by 30 to make rotation slower and less unwieldy
    
    goalAngle += joyZ;

    if (goalAngle >= 360) 
      goalAngle -= 360;       //adjust goalAngle to wrap around to 0 from 360 and vice versa
    if (goalAngle <= 0)
      goalAngle = 360 - fabs(goalAngle);
    
    double vel = (sqrt((joyX * joyX) + (joyY * joyY)) / M_SQRT2); //get velocity value out of joystick values

    double x2 = vel * (dcos(datan2(joyY, joyX) - gyroAngle));     //i believe these generate coordinates based off the joystick
    double y2 = vel * (dsin(datan2(joyY, joyX) - gyroAngle));     //values. they are used to calculate the direction the robot should move.

    if (x2 == 0) 
      x2 = 0.0001; //safeguard against x2 being zero so no errors occur.
    
    double datanx2y2 = datan(y2/x2);            //save the code from calculating this every single run of the for loop
    double sqrtx2y2 = sqrt((x2*x2) + (y2*y2));  //^^ i believe this line finds the length of the segment made by the joystick to the home position
    
    for(int i = 0, addAngle = 45; i <= 3; i++, addAngle += 90)  //calculates motor speeds and puts them into an array
      motorSpeeds[i] = -dsin(datanx2y2 + addAngle) * sqrtx2y2;  //add angle offsets each motor calc by 90 degrees because the wheels are offset themselves

    if (x2 < 0)  //Inverts the motors when x2 is less than 0 to account for the nonnegative sine curve
      for(int i = 0; i <= 3; i++)
        motorSpeeds[i] *= -1;

    previousAngle = angleError; //store previous error value for use in derivative

    angleError = (gyroAngle - goalAngle); //difference between the current angle and the goal angle
    
    if (angleError > 180)                               //adjust the angle error to force the pid to follow the shortest
      angleError = -((360 - gyroAngle) + goalAngle);    //route to the goal.
    
    if (angleError < -180)                              //^^
      angleError = (360 - goalAngle) + gyroAngle; 
    
    if (fabs(angleError) < 10)                          //if the angle error is small enough, activate the integral
      angleIntegral += angleError; 
    else 
      angleIntegral = 0;                                //set it to 0 if it's too big

    angleDerivative = previousAngle - angleError;       //calculation of derivative pid value

    double turnValue = (angleError*kP) + (kI*angleIntegral) + (kD*angleDerivative); //final pid calculation

    for(int i = 0; i <= 3; i++) 
      motorSpeeds[i] -= turnValue;
    
    double maxAxis = MAX(fabs(joyX), fabs(joyY), fabs(angleError)); //Find the maximum input given by the controller's axes and the angle corrector
    double maxOutput = MAX(fabs(motorSpeeds[0]), fabs(motorSpeeds[1]), fabs(motorSpeeds[2]), fabs(motorSpeeds[3])); //Find the maximum output that the drive program has calculated
      
    if (maxOutput == 0 || maxAxis == 0)
      normalizer = 0; //Prevent the undefined value for normalizer
    else
      normalizer = maxAxis / maxOutput; //calculate normalizer

    for (int i = 0; i <= 3; i++) 
      motorSpeeds[i] *= normalizer; //caps motor speeds to 100 without losing the ratio between each value

    backLeft.spin(forward, motorSpeeds[_bL_], percent);
    backRight.spin(forward, motorSpeeds[_bR_], percent);    //spin the motors at their speeds.
    frontRight.spin(forward, motorSpeeds[_fR_], percent);
    frontLeft.spin(forward, motorSpeeds[_fL_], percent);
    
    //need TESTING ON AN ACTUAL ROBOT TOPAJ{EPOTJH{PIOADTIPH{BIPDT} hzh aey5hye6q565q63563hfrfxtgsfrzrxtgfrzt4exrfe4zr4frf4e34re3f4s3er545re3fr3f54re3
    //i got testing:)

    //everything below is for debugging, just printing the variables to the brain screen
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(5, 5);

    Brain.Screen.print("fL: ");
    Brain.Screen.print(motorSpeeds[_fL_]);
    Brain.Screen.newLine();
    
    Brain.Screen.print("fR: ");
    Brain.Screen.print(motorSpeeds[_fR_]);
    Brain.Screen.newLine();
    
    Brain.Screen.print("bR: ");
    Brain.Screen.print(motorSpeeds[_bR_]);
    Brain.Screen.newLine();
    
    Brain.Screen.print("bL: ");
    Brain.Screen.print(motorSpeeds[_bL_]);
    Brain.Screen.newLine();
  
    Brain.Screen.print("gA: ");
    Brain.Screen.print(goalAngle);
    Brain.Screen.newLine();
    
    Brain.Screen.print("gyro: ");
    Brain.Screen.print(gyroAngle);
    Brain.Screen.newLine();

    Brain.Screen.print("error: ");
    Brain.Screen.print(angleError);
    Brain.Screen.newLine();

    if (Controller1.ButtonR1.pressing())    //brings ball straight up and into tower
    {
      leftFlipOut.spin(forward, 100, percent);
      rightFlipOut.spin(forward, 100, percent);
      middleIntake.spin(forward, 100, percent);
      finalIntake.spin(forward, 100, percent);
    }
    else if(Controller1.ButtonL1.pressing()) //brings ball to hoarder cell, by spionning the roler above it backwards
    {
      leftFlipOut.spin(forward, 100, percent);
      rightFlipOut.spin(forward, 100, percent);
      middleIntake.spin(forward, 100, percent);
      finalIntake.spin(forward, -100, percent);
    }
    else if(Controller1.ButtonR2.pressing()) //brings balls up, does not shoot them out
    {
      leftFlipOut.spin(forward, 100, percent);
      rightFlipOut.spin(forward, 100, percent);
      middleIntake.spin(forward, 100, percent);
      finalIntake.stop(coast);
    }
    else if (Controller1.ButtonL2.pressing()) //outakes balls out the intake
    {
      leftFlipOut.spin(forward, -100, percent);
      rightFlipOut.spin(forward, -100, percent);
      middleIntake.spin(forward, -100, percent);
      finalIntake.stop(coast);
    }
    else //stops all intake motors
    {
      leftFlipOut.stop(coast);
      rightFlipOut.stop(coast);
      middleIntake.stop(coast);
      finalIntake.stop(coast);
    }
    wait(20, msec); 
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
