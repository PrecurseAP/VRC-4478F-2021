/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Aiden Pringle (4478F)                                     */
/*    Created:      Tues Dec 7 2021                                           */
/*    Description:  4478F Robot Code v2 Main File                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// mLUpper              motor         12              
// mLLower              motor         11              
// mRUpper              motor         15              
// mRLower              motor         19              
// GPS                  gps           16              
// mConveyor            motor         18              
// mLTray               motor         1               
// mArm                 motor         13              
// clawPiston           digital_out   F               
// GYRO                 inertial      17              
// mRTray               motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "utils.h"
#include "movements.h"


using namespace vex;

// A global instance of competition
competition Competition;

void pre_auton(void) {
  vexcodeInit();

}

void autonomous(void) {

}

brakeType lockDrive = coast;

void lockDriveHold() {
  lockDrive = hold;
}

void lockDriveCoast() {
  lockDrive = coast;
}

void usercontrol(void) {

  /*GPS.startCalibration();
  GYRO.startCalibration();
  while(GPS.isCalibrating() || GYRO.isCalibrating()) {
    wait(100, msec);
  }
  wait(2000, msec);
*/
  Controller1.ButtonA.pressed(clawToggle); //little callback for toggling the claw pneumatics

  Controller1.ButtonX.pressed(lockDriveHold); //callbacks for toggling the locking drive code
  Controller1.ButtonY.pressed(lockDriveCoast);

 // turnToAngle(180, 1500, .662, 0.00000, 0.08);

  //float gAngle = sin(Brain.timer(msec)/500.0);
  //float gAngle = GYRO.heading(degrees);
  //Graph graph = Graph(gAngle, 200);

  while (1) {
    //gAngle = 20 * sin(Brain.timer(msec)/500.0);
    //gAngle = GYRO.heading(degrees);
    //graph.updateData(gAngle);
    //graph.drawGraph();

    float LSpeed = logDrive(Controller1.Axis3.position(percent)*(12.0/100.0));
    float RSpeed = logDrive(Controller1.Axis2.position(percent)*(12.0/100.0));

    mLUpper.spin(forward, LSpeed, volt);
    mLLower.spin(forward, LSpeed, volt);
    mRUpper.spin(forward, RSpeed, volt);
    mRLower.spin(forward, RSpeed, volt);

    
    if (LSpeed == 0) {
      mLLower.stop(lockDrive);
      mLUpper.stop(lockDrive);
    }
    if (RSpeed == 0) {
      mRLower.stop(lockDrive);
      mRUpper.stop(lockDrive);
    }

    //ring conveyor
    if (Controller1.ButtonL1.pressing()) {
      mConveyor.spin(forward, 180, rpm);
    } else if (Controller1.ButtonL2.pressing()){
      mConveyor.spin(reverse, 180, rpm);
    } else {
      mConveyor.stop(hold);
    }

    //arm
    if (Controller1.ButtonR1.pressing()) {
      mArm.spin(forward, 100, percent);
    } else if (Controller1.ButtonR2.pressing()) {
      mArm.spin(reverse, 100, percent);
    } else {
      mArm.stop(hold);
    }

    //MOGO Tray
    if(Controller1.ButtonUp.pressing()){
      mLTray.spin(forward, 100, percent);
      mRTray.spin(forward, 100, percent);
    } else if (Controller1.ButtonDown.pressing()){
      mLTray.spin(reverse, 100, percent);
      mRTray.spin(reverse, 100, percent);
    } else {
      mLTray.stop(hold);
      mRTray.stop(hold);
    }

    wait(15, msec);
  }
}

int main() {

  Brain.Screen.render(true, false);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
