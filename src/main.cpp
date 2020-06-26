#include "vex.h"
#include "trig.h"
#include "custommath.h"

#define _bL_ 0
#define _bR_ 1
#define _fR_ 2
#define _fL_ 3

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// GYRO                 inertial      11              
// Controller1          controller                    
// frontLeft            motor         1               
// frontRight           motor         2               
// backLeft             motor         3               
// backRight            motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

competition Competition;

void stopAllDrive() {
  backLeft.stop();
  backRight.stop();
  frontLeft.stop();
  frontRight.stop();
}

void resetGyro() {
  stopAllDrive();
  GYRO.calibrate();
}
void pre_auton(void) {

  vexcodeInit();
}

void autonomous(void) {

}

void usercontrol(void) {
  resetGyro();
  wait(2000, msec);
  Controller1.ButtonA.pressed(resetGyro);

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

    if (goalAngle >= 360) {
      goalAngle -= 360;
    }                                       //adjust goalAngle to wrap around to 0 from 360 and vice versa
    if (goalAngle <= 0) {
      goalAngle = 360 - fabs(goalAngle);
    }
    //
    double vel = (sqrt((joyX * joyX) + (joyY * joyY)) / M_SQRT2); //get velocity value out of joystick values

    double x2 = vel * (dcos(datan2(joyY, joyX) - gyroAngle));
    double y2 = vel * (dsin(datan2(joyY, joyX) - gyroAngle));

    if (x2 == 0) 
      x2 = 0.0001; //safeguard against x2 being zero so no errors occur.
    
    double datanx2y2 = datan(y2/x2);
    double sqrtx2y2 = sqrt((x2*x2) + (y2*y2));
    
    for(int i = 0, addAngle = 45; i <= 3; i++, addAngle += 90)
      motorSpeeds[i] = -dsin(datanx2y2 + addAngle) * sqrtx2y2;

    /*double bL = -dsin(datanx2y2 + 45) * sqrtx2y2; //Set the motors to their appropriate speed based on the formula (each are offset by a factor of pi/2 to account for the 90 degree difference in the wheels)
    double bR = -dsin(datanx2y2 + 135) * sqrtx2y2;
    double fR = -dsin(datanx2y2 + 225) * sqrtx2y2;
    double fL = -dsin(datanx2y2 + 315) * sqrtx2y2;*/

    if (x2 < 0)  //Inverts the motors when x2 is less than 0 to account for the nonnegative sine curve
      for(int i = 0; i <= 3; i++)
        motorSpeeds[i] *= -1;
      /*fL *= -1;
      fR *= -1;
      bR *= -1;
      bL *= -1;*/

    previousAngle = angleError;

    angleError = (gyroAngle - goalAngle);
    
    if (angleError > 180) 
      angleError = -((360 - gyroAngle) + goalAngle);
    
    if (angleError < -180)  
      angleError = (360 - goalAngle) + gyroAngle; 
    
    if (fabs(angleError) < 10)  
      angleIntegral += angleError; 

    angleDerivative = previousAngle - angleError;

    double turnValue = (angleError*kP) + (kI*angleIntegral) + (kD*angleDerivative);

    for(int i = 0; i <= 3; i++) 
      motorSpeeds[i] -= turnValue;
    
    /*fL -= turnValue; //Include the value of the turning axis in the output.
    fR -= turnValue;
    bR -= turnValue;
    bL -= turnValue;*/
    
    double maxAxis = MAX(fabs(joyX), fabs(joyY), fabs(angleError)); //Find the maximum input given by the controller's axes and the angle corrector
    double maxOutput = MAX(fabs(motorSpeeds[0]), fabs(motorSpeeds[1]), fabs(motorSpeeds[2]), fabs(motorSpeeds[3])); //Find the maximum output that the drive program has calculated
      
    if (maxOutput == 0 || maxAxis == 0)
      normalizer = 0; //Prevent the undefined value for normalizer
    else
      normalizer = maxAxis / maxOutput;

    for (int i = 0; i <= 3; i++) 
      motorSpeeds[i] *= normalizer;
    /*fL *= normalizer;
    fR *= normalizer;
    bR *= normalizer;
    bL *= normalizer;*/

    backLeft.spin(forward, motorSpeeds[_bL_], percent);
    backRight.spin(forward, motorSpeeds[_bR_], percent);
    frontRight.spin(forward, motorSpeeds[_fR_], percent);
    frontLeft.spin(forward, motorSpeeds[_fL_], percent);
    
    //need TESTING ON AN ACTUAL ROBOT TOPAJ{EPOTJH{PIOADTIPH{BIPDT} hzh aey5hye6q565q63563hfrfxtgsfrzrxtgfrzt4exrfe4zr4frf4e34re3f4s3er545re3fr3f54re3
    //i got testing:)

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
