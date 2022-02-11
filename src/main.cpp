// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// mFrontLeft           motor         1               
// mMidLeft             motor         2               
// mBackLeft            motor         3               
// mFrontRight          motor         11              
// mMidRight            motor         12              
// mBackRight           motor         13              
// mLift                motor         18              
// mConveyor            motor         19              
// Controller2          controller                    
// tilterPiston         digital_out   A               
// clawPiston           digital_out   H               
// GYRO                 inertial      10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "utils.h"
#include "movements.h"

using namespace vex;

competition Competition;

template <typename T> 
int sgn(T val) { //SIGNUM
    return (T(0) < val) - (val < T(0));
}

template <typename S>
S logDriveVolt(S s) {
  return pow(12, 2.6) / pow(12, 1.6) * sgn(s);
}

int autonSelection = 0;
std::string autonPath = "Full AWP";

void cycleAuton() {
  if (autonSelection < 7) {
    autonSelection++;
  } else {
    autonSelection = 0;
  }

  switch (autonSelection) {
    case 0: {
      autonPath = "RIGHT 20p + rings";
      break;
    }
    case 1: {
      autonPath = "LEFT 20P";
      break;
    }
    case 2: {
      autonPath = "RIGHT SIDE 40 WITH RINGS";
      break;
    }
    case 3: {
      autonPath = "left center goal dash!!!";
      break;
    }
    case 4: {
      autonPath = "FULL AWP!! (RIGHT SIDE)";
      break;
    }
    case 5: {
      autonPath = "right side center sprint";
      break;
    }
    case 6: {
      autonPath = "left 20";
      break;
    }
    case 7: {
      autonPath = "SKILLS";
      break;
    }
    default: {
      autonPath = "something happened, restart the program :)";
      break;
    }
  }
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(autonPath.c_str());
  Brain.Screen.render();
}

void pre_auton(void) {
  vexcodeInit();

  clawPiston.set(false);
  tilterPiston.set(false);

  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(autonPath.c_str());
  Brain.Screen.render();

  Brain.Screen.pressed(cycleAuton);

  GYRO.startCalibration();
  while(GYRO.isCalibrating()) {
    wait(100, msec);
  }
  wait(2500, msec);

}

void autonomous(void) {
  thread deploy_;
  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mBackLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);
  mLift.setPosition(0, degrees);
  mConveyor.setPosition(0, degrees);
  
  autonSelection = 8;

  switch(autonSelection) {
    case 0: /*right 20*/ {
      tilterPiston.set(true);
      clawPiston.set(true);
      moveStraight(44, 1200, 5.9);
      clawPiston.set(false);
      wait(75, msec);
      moveStraight(-30, 1500);

      //turn to alliance goal
      raiseLift(100, true);
      turnWithClawGoal(270, 1500);
      clawPiston.set(true);
      wait(50, msec);
      moveStraight(-10, 1500);
      moveStraight(-3, 500, 5.50);
      tilterPiston.set(false);
      
      raiseLift(200, true);
      moveStraight(8, 1000);
      turnWithTilterGoal(0, 1500);

      mConveyor.spin(forward, 600, percent);
      basicDrive(30);
      wait(1700, msec);
      moveStraight(-35, 1500);

      break;
    }
    case 1: /*left 20*/ {
      clawPiston.set(true);
      moveStraight(45, 1250, 5.92);
      clawPiston.set(false);
      wait(50, msec);
      moveStraight(-35, 1500);
      break;
    }
    case 2: /*right 40 no ring*/ {
      int startTime = Brain.timer(msec);
      tilterPiston.set(true);
      clawPiston.set(true);
      moveStraight(45, 1200, 5.9);
      clawPiston.set(false);
      wait(75, msec);
      raiseLift(60, false);
      moveStraight(-34, 1500);
      turnWithClawGoal(140, 1500);
      clawPiston.set(true);
      raiseLift(0, false);
      moveStraight(-20, 1000);
      turnToAngle(320, 1000);
      moveStraight(17, 750, 5.2);
      clawPiston.set(false);
      raiseLift(60, false);
      moveStraight(-39, 1200);
      turnWithClawGoal(270, 1000);
      raiseLift(0, true);
      clawPiston.set(true);
      moveStraight(-21, 1000);
      moveStraight(-3, 500);
      tilterPiston.set(false);
      wait(200, msec);
      raiseLift(200, false);
      moveStraight(9, 500, 5.5);
      turnWithClawGoal(0, 800);
      mConveyor.spin(forward, 600, percent);
      basicDrive(30);
      wait(1700, msec);
      moveStraight(-35, 1500);

      std::cout << "Auton completed in " << Brain.timer(msec) - startTime << " ms." << std::endl;
      break;
    }
    case 3: /*left center dash*/ {
      int startTime = Brain.timer(msec);
      //mConveyor.spin(forward, 300, percent);

      clawPiston.set(true);
      tilterPiston.set(true);
      moveStraight(35, 1250, 5.82);
      turnToAngle(245, 1500);
      moveStraight(-20, 1500);
      turnToAngle(69, 1500);
      moveStraight(17, 800, 5.50);
      clawPiston.set(false);
      wait(200, msec);
      turnWithClawGoal(45, 1500);
      moveStraight(-48, 1500);
      clawPiston.set(true);
      wait(100, msec);
      turnToAngle(330, 1500);
      moveStraight(-10, 1500);
      tilterPiston.set(false);
      mConveyor.spin(forward, 550, rpm);
      
      /*turnToAngle(270, 1500);
      moveStraight(15, 1000, 5);
      wait(100,msec);
      clawPiston.set(false);
      wait(100,msec);
      moveStraight(-9, 1000);
      turnWithClawGoal(0, 1500);
      moveStraight(32, 1200);
      raiseLift(500, true);
      turnWithClawGoal(270, 1500);
      moveStraight(5, 1000);
      raiseLift(350, true);
      clawPiston.set(true);
      moveStraight(-5, 1000);
      */
      std::cout << "Auton completed in " << Brain.timer(msec) - startTime << " ms." << std::endl;
      break;
    }
    case 4: /* full awp*/ {
      int startTime = Brain.timer(msec);

      clawPiston.set(true);
      tilterPiston.set(true);

      moveStraight(44.5, 1200, 5.9);
      clawPiston.set(false);
      wait(75, msec);
      moveStraight(-28, 1500);
      raiseLift(100, true);
      turnWithClawGoal(273, 2000);
      moveStraight(-9, 1200, 5.5);
      moveStraight(-3, 500, 5);
      tilterPiston.set(false);
      mConveyor.spin(forward, 300, rpm);
      wait(300, msec);
      mConveyor.stop(hold);
      moveStraight(20, 1500);
      tilterPiston.set(true);
      moveStraight(77, 2000);
      turnWithClawGoal(326, 1500);
      raiseLift(0, true);
      clawPiston.set(true);
      wait(50, msec);
      moveStraight(-25, 1500);
      tilterPiston.set(false);
      mConveyor.spin(forward, 250, rpm);
      wait(750, msec);
      mConveyor.stop(hold);

      std::cout << "Auton completed in " << Brain.timer(msec) - startTime << " ms." << std::endl;
      break;
    }
    case 5: /* right center dash + rings*/ {
      int startTime = Brain.timer(msec);

      clawPiston.set(true);
      tilterPiston.set(true);

      moveStraight(17.5, 1000);
      turnToAngle(315, 1000);

      moveStraight(38.5, 1500, 5.5);

      clawPiston.set(false);
      wait(100, msec);

      raiseLift(50, true);
      moveStraight(-40, 1500);

      turnWithClawGoal(270, 1500);

      clawPiston.set(true);

      wait(50, msec);

      moveStraight(-18, 1500);
      moveStraight(-3, 500);

      tilterPiston.set(false);

      moveStraight(11, 1000);

      turnWithTilterGoal(0, 1500);

      raiseLift(200, true);
      mConveyor.spin(forward, 600, percent);
      basicDrive(30);
      wait(1700, msec);
      moveStraight(-35, 1500);

      std::cout << "Auton completed in " << Brain.timer(msec) - startTime << " ms." << std::endl;
      break;
    }
    case 6: /* Left 20 with awp */ {
      int startTime = Brain.timer(msec);      
      
      clawPiston.set(true);
      tilterPiston.set(true);
      moveStraight(47, 1250, 5.82);
      clawPiston.set(false);
      wait(50, msec);
      moveStraight(-42, 1500);

      clawPiston.set(true);
      turnToAngle(280, 1500);
      moveStraight(-10, 1500);
      tilterPiston.set(false);
      wait(100, msec);
      mConveyor.spin(forward, 550, rpm);
      wait(1000, msec);
      mConveyor.stop();

      std::cout << "Auton completed in " << Brain.timer(msec) - startTime << " ms." << std::endl;
      break;
    }
    case 7: /* skills */ {
      clawPiston.set(true);
      tilterPiston.set(true);

      moveStraight(5, 700, 5.3);

      clawPiston.set(false);
      raiseLift(500, true);
      moveStraight(-2, 600);
      turnWithClawGoal(93, 2500);
      moveStraight(-15, 1000, 5.1);
      raiseLift(100, true);
      moveStraight(-29, 3500);
      moveStraight(-3, 500);
      moveStraight(-3, 500);
      tilterPiston.set(false);
      wait(100, msec);
      moveStraight(-44.5, 1500);
      turnToAngle(2, 2000, .62);
      moveStraight(3, 500, 5.1);
      wait(300, msec);
      tilterPiston.set(true);
      wait(300, msec);
      moveStraight(23, 1000, 5.5);
      raiseLift(385, true);
      turnWith2Goals(270, 1500);
      moveStraight(8, 1000, 4.5);
      wait(100, msec);
      //raiseLift(, true);
      clawPiston.set(true);
      moveStraight(-5.5, 1000);
      turnToAngle(185, 1500);
      raiseLift(0, false);
      moveStraight(38, 1500, 5.35);
      clawPiston.set(false);
      wait(200, msec);
      turnWithClawGoal(7, 1500);
      moveStraight(-7, 1200, 5);
      tilterPiston.set(false);
      wait(100, msec);
      moveStraight(3, 500);
      turnWith2Goals(4, 2000);
      moveStraight(47, 1500);
      //tilterPiston.set(true);
      //wait(300, msec);
      raiseLift(500, true);
      turnWith2Goals(270, 2000);
      moveStraight(5, 1000, 5.5);
      turnWithClawGoal(270, 1500);
      moveStraight(5, 1000, 5.5);
      raiseLift(350, true);
      clawPiston.set(true);
      wait(200, msec);
      moveStraight(-5, 1000, 5.5);
      turnToAngle(0, 1000);
      raiseLift(0, true);
      moveStraight(31, 1500, 5.4);
      turnWithTilterGoal(90, 1500);

      //raise the lift a bit, then clear the front of the bot of rings with the conveyor
      raiseLift(200, true);
      mConveyor.spin(forward, 320, rpm);
      moveStraight(11, 2000, 5.5);
      mConveyor.stop();     
      raiseLift(0, true);
      turnWithTilterGoal(90, 1500);
      moveStraight(14, 1000, 5);
      clawPiston.set(false);
      wait(50, msec);


      raiseLift(100, true);
      turnWith2Goals(128, 1500); //turn towards the secondary platform
      moveStraight(35, 1500);
      raiseLift(450, true);
      moveStraight(25, 700, 5.5);
      clawPiston.set(true);
      wait(200, msec);
      moveStraight(-8, 900);
      turnWithTilterGoal(355, 1000);
      raiseLift(0, true);
      moveStraight(32, 1500, 5.5);
      clawPiston.set(false);
      wait(50, msec);
      raiseLift(100, true);
      moveStraight(-5, 700);
      turnWith2Goals(180, 1500);
      break;
    }
  }

}

bool tilterToggled = false;

void toggleTilter() {
  tilterPiston.set(!tilterToggled);
  tilterToggled = !tilterToggled;
}

bool clawToggled = false;

void toggleClaw() {
  clawPiston.set(!clawToggled);
  clawToggled = !clawToggled;
}

brakeType lockDrive = coast;

void lockDriveHold() {
  lockDrive = hold;
}

void lockDriveCoast() {
  lockDrive = coast;
}

void usercontrol(void) {

  Controller1.ButtonR2.pressed(toggleTilter);
  Controller1.ButtonR1.pressed(toggleClaw);

  Controller1.ButtonX.pressed(lockDriveHold); //callbacks for toggling the locking drive code
  Controller1.ButtonY.pressed(lockDriveCoast);

  while (1) {
    float LSpeed = logDriveVolt(Controller1.Axis3.position(percent)*(float)(12.0/100.0));
    float RSpeed = logDriveVolt(Controller1.Axis2.position(percent)*(float)(12.0/100.0));

    if (LSpeed != 0) {
      mFrontLeft.spin(forward, LSpeed, volt);
      mMidLeft.spin(forward, LSpeed, volt);
      mBackLeft.spin(forward, LSpeed, volt);
    }
    if (RSpeed != 0) {
      mFrontRight.spin(forward, RSpeed, volt);
      mMidRight.spin(forward, RSpeed, volt);
      mBackRight.spin(forward, RSpeed, volt);
    }

    if (LSpeed == 0) {
      mFrontLeft.stop(lockDrive);
      mMidLeft.stop(lockDrive);
      mBackLeft.stop(lockDrive);
    }
    if (RSpeed == 0) {
      mFrontRight.stop(lockDrive);
      mMidRight.stop(lockDrive);
      mBackRight.stop(lockDrive);
    }

    if (Controller1.ButtonL1.pressing()) {
      mLift.spin(forward, 100, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      mLift.spin(reverse, 100, percent);
    } else {
      mLift.stop(hold);
    }

    if (Controller2.ButtonR1.pressing()) {
      mConveyor.spin(forward, 600, rpm);
    } else if (Controller2.ButtonR2.pressing()) {
      mConveyor.spin(reverse, 600, rpm);
    } else {
      mConveyor.stop(coast);
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
