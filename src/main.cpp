#include "vex.h"
#include "trig.h"
#include "custommath.h"
#include "odom.h"

//define macros for indices of motor speeds in main speed array
#define _bL_ 0
#define _bR_ 1
#define _fR_ 2    
#define _fL_ 3

using namespace vex;

competition Competition;

void stopAllDrive(brakeType bT) {
  /** 
   * Stops all motors with a specific brakeType. Parameter must be given.
   */
  backLeft.stop(bT);
  backRight.stop(bT);
  frontLeft.stop(bT);
  frontRight.stop(bT);
}

void resetGyro() {
  /**
   * Stops all motors then calibrates the gyro.
   */
  stopAllDrive(hold);
  GYRO.calibrate();
}

void drawGoal(int x, int y, color ballC) {
  Brain.Screen.drawCircle(x, y, 9, color::transparent);
  Brain.Screen.drawCircle(x, y, 8, color::transparent);
  Brain.Screen.drawCircle(x, y, 7, color::transparent);
  Brain.Screen.drawCircle(x, y, 6, ballC);
}

void pre_auton(void) {
  /*    -The pre_auton function draws the field diagram on the screen and sets up the auton path-*/

  bool autonSelected = false;
  while(!autonSelected) {
    
    Brain.Screen.setPenColor("#777777");
    
    for (int ROW = 0; ROW < 6; ROW++) 
      for (int COL = 0; COL < 6; COL++) 
        Brain.Screen.drawRectangle( (COL * 35) + 130 , (ROW * 35) + 18 , 35 , 35 , "#888888");
    //^^ The above nested loop generates the 6x6 grid of field tiles in gray

    Brain.Screen.setPenColor(color::red);
    Brain.Screen.drawRectangle(110, 20, 3, 205, color::red);   //draw leftmost red line

    Brain.Screen.setPenColor(color::blue);
    Brain.Screen.drawRectangle(360, 20, 3, 205, color::blue);  //draw rightmost blue line 

    Brain.Screen.setPenColor(color::white);

    Brain.Screen.drawRectangle(130, 120, 35, 2);               //draw two horizontal white lines on the left
    Brain.Screen.drawRectangle(130, 124, 35, 2);               //
                                                
    Brain.Screen.drawRectangle(164, 18, 2, 209);               //draw vertical white line on the left

    Brain.Screen.drawRectangle(232, 18, 2, 209);               //draw two vertical white lines in the center                                  
    Brain.Screen.drawRectangle(236, 18, 2, 209);               //

    Brain.Screen.drawRectangle(304, 18, 2, 209);               //draw vertical white line on the right

    Brain.Screen.drawRectangle(305, 120, 34, 2);               //draw two horizontal white lines on the right                                                                                    
    Brain.Screen.drawRectangle(305, 124, 34, 2);               //

    Brain.Screen.setPenColor("#444444");

    Brain.Screen.drawRectangle(130, 18, 209, 2); //
                                                 //
    Brain.Screen.drawRectangle(130, 226, 209, 2);// 
                                                 //    Draw field border walls in dark gray
    Brain.Screen.drawRectangle(130, 18, 2, 209); //
                                                 //
    Brain.Screen.drawRectangle(338, 18, 2, 209); //

    Brain.Screen.setPenColor("#222222");

    drawGoal(141, 29, color::blue);              //                
    drawGoal(235, 29, color::red);               //draw top three goals
    drawGoal(328, 29, color::red);               //

    drawGoal(141, 123, color::blue);             // 
    drawGoal(235, 123, color::transparent);      //draw middle three goals
    drawGoal(328, 123, color::red);              //

    drawGoal(141, 216, color::blue);             // 
    drawGoal(235, 216, color::blue);             //draw bottom three goals
    drawGoal(328, 216, color::red);              //

    Brain.Screen.drawCircle(152, 40, 6, color::red);

    Brain.Screen.drawCircle(317, 40, 6, color::blue);

    Brain.Screen.drawCircle(317, 205, 6, color::blue);

    Brain.Screen.drawCircle(152, 205, 6, color::red);

    Brain.Screen.drawCircle(235, 70, 6, color::red);          //DRAW ALL THE DAMN BALLS )that arent in goals(

    Brain.Screen.drawCircle(235, 175, 6, color::blue);

    Brain.Screen.drawCircle(235, 107, 6, color::red);

    Brain.Screen.drawCircle(235, 139, 6, color::blue);

    Brain.Screen.drawCircle(219, 123, 6, color::blue);

    Brain.Screen.drawCircle(251, 123, 6, color::red);

    wait(50, msec);
    
  }
}

void autonomous(void) {

}

void usercontrol(void) {
  /**
   * All of our drive code, used to move the robot around.
   */
  Controller1.ButtonA.pressed(resetGyro); //bind a button to reset the gyro

  double goalAngle = 0, angleIntegral = 0, angleError = 0;
  double normalizer, angleDerivative, previousAngle;
  double kP = 1.5, kI = 0.015, kD = 0.8;
  double motorSpeeds[4];

  while (1) {
    double gyroAngle = GYRO.heading();       //grab and store the gyro value

    double joyX = Controller1.Axis4.position();       // Set variables for each joystick axis
    double joyY = -Controller1.Axis3.position();      // joyY is negative because driving would be backwards otherwise
    double joyZ = Controller1.Axis1.position()/30.0;    // this here is divided by 30 to make rotation slower and less unwieldy
    
    goalAngle += joyZ; //shift the goal angle when the right joystick is tilted

    if (goalAngle >= 360) 
      goalAngle -= 360;       //adjust goalAngle to wrap around to 0 from 360 and vice versa
    if (goalAngle <= 0)
      goalAngle = 360 - fabs(goalAngle);
    
    double vel = (sqrt((joyX * joyX) + (joyY * joyY)) / M_SQRT2); //get velocity value out of joystick values

    double x2 = vel * (dcos(datan2(joyY, joyX) - gyroAngle));     //i believe these generate coordinates based off the joystick
    double y2 = vel * (dsin(datan2(joyY, joyX) - gyroAngle));     //values. they are used to calculate the direction the robot should move by simulating a graph


    if (x2 == 0) 
      x2 = 0.0001; //safeguard against x2 being zero so no errors occur.
    
    double datanx2y2 = datan(y2/x2);            //save the code from calculating this every single run of the for loop
    double sqrtx2y2 = sqrt((x2*x2) + (y2*y2));  //^^ i believe this line finds the length of the segment made by the joystick to the home position
    
    for(int i = 0, addAngle = 45; i <= 3; i++, addAngle += 90)  //calculates motor speeds and puts them into an array
      motorSpeeds[i] = -dsin(datanx2y2 + addAngle) * sqrtx2y2;  //add angle offsets each motor calc by 90 degrees because the wheels are offset themselves

    if (x2 < 0)                                         //Inverts the motors when x2 is less than 0 to account for the nonnegative sine curve
      for(int i = 0; i <= 3; i++)
        motorSpeeds[i] *= -1;

    previousAngle = angleError;                         //store previous error value for use in derivative

    angleError = gyroAngle - goalAngle;               //difference between the current angle and the goal angle
    
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
    backRight.spin(forward, motorSpeeds[_bR_], percent);    //spin the motors at their calculated/stored speeds.
    frontRight.spin(forward, motorSpeeds[_fR_], percent);
    frontLeft.spin(forward, motorSpeeds[_fL_], percent);

    if (Controller1.ButtonR1.pressing()) {     //brings ball straight up and into tower
      leftFlipOut.spin(forward, 100, percent);
      rightFlipOut.spin(forward, 100, percent);
      middleIntake.spin(forward, 100, percent);
      finalIntake.spin(forward, 100, percent);
    }
    else if(Controller1.ButtonL1.pressing()) { //brings ball to hoarder cell, by spinning the roller above it backwards
      leftFlipOut.spin(forward, 100, percent);
      rightFlipOut.spin(forward, 100, percent);
      middleIntake.spin(forward, 100, percent);
      finalIntake.spin(reverse, 100, percent);
    }
    else if(Controller1.ButtonR2.pressing()) { //bring balls up, does not shoot them out
      leftFlipOut.spin(reverse, 100, percent);
      rightFlipOut.spin(reverse, 100, percent);
      middleIntake.spin(forward, 100, percent);
      finalIntake.stop(coast);
    }
    else if (Controller1.ButtonL2.pressing()) { //outtake balls
      leftFlipOut.spin(reverse, 100, percent);
      rightFlipOut.spin(reverse, 100, percent);
      middleIntake.spin(reverse, 100, percent);
      finalIntake.stop(coast);
    }
    else { //stops all intake motors
      leftFlipOut.stop(coast);
      rightFlipOut.stop(coast);
      middleIntake.stop(coast);
      finalIntake.stop(coast);
    }
     
    wait(10, msec); 
  }
}

int main() {
  vexcodeInit();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
