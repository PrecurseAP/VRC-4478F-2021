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
// limLift              limit         H               
// potTilter            pot           G               
// Vision               vision        14              
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Aiden Pringle (4478F)                                     */
/*    Created:      Tues Dec 7 2021                                           */
/*    Description:  4478F Robot Code v2 Main File                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "utils.h"
#include "movements.h"


using namespace vex;

// A global instance of competition
competition Competition;

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
      autonPath = "Full AWP";
      break;
    }
    case 1: {
      autonPath = "Right side 20 point with rings";
      break;
    }
    case 2: {
      autonPath = "Right Side tall goal FAST";
      break;
    }
    case 3: {
      autonPath = "Right Side 40 pointer + AWP";
      break;
    }
    case 4: {
      autonPath = "Left Side 20 point driver load";
      break;
    }
    case 5: {
      autonPath = "Left side center sprint";
      break;
    }
    case 6: {
      autonPath = "Left Side 40 wit da winpoint";
      break;
    }
    case 7: {
      autonPath = "skills";
      break;
    }
    default: {
      autonPath = "what the hell";
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

  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(autonPath.c_str());
  Brain.Screen.render();

  Brain.Screen.pressed(cycleAuton);

  GYRO.startCalibration();
  while(GYRO.isCalibrating()) {
    wait(100, msec);
  }
  wait(2000, msec);

}

void autonomous(void) {
  mLLower.setPosition(0, degrees);
  mLUpper.setPosition(0, degrees);
  mRLower.setPosition(0, degrees);
  mRUpper.setPosition(0, degrees);
  mArm.setPosition(0, degrees);
  mLTray.setPosition(0, degrees);
  mRTray.setPosition(0, degrees);
  mConveyor.setPosition(0, degrees);

  //autonSelection = 7;

  thread _deploy;

  switch(autonSelection) {
    case 0: /* FULL AWP */ {
      _deploy = thread(deploy);

      moveStraight(-46, 1500, 5.7); //move forward to first mogo

      clawToggle(); //grip
      wait(100, msec);

      raiseLift(150, false);
      moveStraight(28, 1100); //lift goal and swiftly line up to alliance goal

      turnWithClawGoal(270, 1050); //turn to face alliance mogo

      moveStraight(-6, 700); //inch backward to not hit mogo

      lowerTilter(); 
      moveStraight(15, 800); //lower tilter and then drive into alliance goal

      raiseTilterWithGoal();

      thread dep = thread(depositAndDrop);

      moveStraight(-95, 2600);

      //lowerLift();

      //clawToggle();
      
      //moveStraight(0, 500);

      turnWithClawGoal(15, 1200);

      moveStraight(30, 1000);
      
      clawToggle();

      lowerTilter(90, -540, false);

      turnToAngle(276, 1000);

      moveStraight(15, 900);

      raiseTilterWithGoal();

      spinConveyor();

      break;
    }
    case 1: /* Right 20 w/ Rings*/ {
      _deploy = thread(deploy);

      moveStraight(-46, 1500, 5.9); //move forward to first mogo

      clawToggle(); //grip
      wait(200, msec);

      raiseLift(150, false);
      moveStraight(28, 1100); //lift goal and swiftly line up to alliance goal

      turnWithClawGoal(271, 1100); //turn to face alliance mogo

      clawToggle();
      wait(200, msec);

      moveStraight(-6, 700); //inch backward to not hit mogo

      lowerTilter(); 
      moveStraight(15, 800); //lower tilter and then drive into alliance goal

      raiseTilterWithGoal();

      moveStraight(-7, 800);

      turnWithTilterGoal(181, 1500);

      spinConveyor();

      moveLeftSide(40);
      moveRightSide(40);

      wait(2200, msec);

      //stopAllDrive(hold);

      moveStraight(-50, 2000);
      mConveyor.stop();

      break;

    }
    case 2: /* Right super fast TALL MAN */ {
      _deploy = thread(deploy);
      
      moveStraight(-13, 1000);

      turnToAngle(324, 900);

      moveStraight(-44, 1500, 5.9);

      clawToggle();

      wait(200, msec);

      raiseLift(100, false);

      turnWithClawGoal(324, 1500);

      moveStraight(43, 1500);

      turnWithClawGoal(270, 1500);

      lowerTilter();

      clawToggle();
      wait(200, msec);

      moveStraight(13, 1100);

      raiseTilterWithGoal();

      moveStraight(-9, 600);

      turnWithTilterGoal(180, 1000);

      spinConveyor();

      moveLeftSide(40);
      moveRightSide(40);

      wait(2200, msec);

      //stopAllDrive(hold);

      moveStraight(-50, 2000);
      mConveyor.stop();

      break;
    }
    case 3: /* Right 40 point + AWP */{
      _deploy = thread(deploy);

      moveStraight(-46, 1500, 5.9); //move forward to first mogo

      clawToggle(); //grip
      wait(200, msec);

      raiseLift(150, false);
      moveStraight(34, 1100); //lift goal and swiftly line up to alliance goal

      turnWithClawGoal(140, 1500);

      moveStraight(-10, 800);

      clawToggle();
      wait(200, msec);
      lowerLift();
      moveStraight(27, 1300);

      turnToAngle(327, 1200);

      moveStraight(-15, 1000, 5.9);

      clawToggle();
      wait(200, msec);

      raiseLift(100, false);

      turnWithClawGoal(335, 700);

      moveStraight(40, 2000);

      turnWithClawGoal(270, 1200);

      clawToggle();
      wait(200, msec);

      lowerTilter(100, -540, true);

      moveStraight(19, 1000);

      raiseTilterWithGoal();

      spinConveyor();

      moveStraight(-20, 1000);

      break;
    }
    case 4: /* left 20 + driver load */ {
      _deploy = thread(deploy);

      moveStraight(-49, 1500, 5.9); //move forward to first mogo

      clawToggle(); //grip
      wait(200, msec);

      raiseLift(150, false);
      moveStraight(31, 1200);

      turnWithClawGoal(45, 1000);

      clawToggle();
      wait(200, msec);

      moveStraight(25, 1300);
      
      turnToAngle(267, 1200);
      lowerTilter();

      moveStraight(16, 1000);
        
      mLTray.setVelocity(100, percent);
      mRTray.setVelocity(100, percent);
      mLTray.spinToPosition(-325, degrees, false);
      mRTray.spinToPosition(-325, degrees, true);

      moveStraight(-22, 1000);

      spinConveyor();

      moveLeftSide(30);
      moveRightSide(30);

      wait(1700, msec);

      stopAllDrive(hold);
      mConveyor.stop(coast);

      break;
    }
    case 5: /* left center sprint */ {
      _deploy = thread(deploy);

      lowerTilter(100, -540, false);

      moveStraight(-18, 1200);

      turnToAngle(230, 1100);

      moveStraight(21, 800);

      turnToAngle(38, 1000);

      moveStraight(-20, 900);

      turnToAngle(44, 1100);

      raiseTilterWithGoal(false);

      moveStraight(-10, 1000, 5.9);

      clawToggle();
      wait(200, msec);

      raiseLift(100, true);

      lowerTilter(100, -540, false);

      moveStraight(45, 1500);

      clawToggle();
      
      wait(200, msec);

      turnToAngle(321, 1500);

      moveStraight(13, 900);
      
      raiseTilterWithGoal();
      wait(300, msec);
      spinConveyor();

      break;
    }
    case 6: /* left side 40 + AWP */ {
      _deploy = thread(deploy);

      moveStraight(-49, 1500, 5.9); //move forward to first mogo

      clawToggle(); //grip
      wait(200, msec);

      raiseLift(150, false);
      moveStraight(31, 1200);



      break;
    }
    case 7: /* prog skills */ {

      //all angles are offset from previous autons by +90 degs

      _deploy = thread(deploy);

      wait(500, msec);

      moveStraight(-4, 1000, 5.9);

      clawToggle();
      wait(200, msec);

      raiseLiftFully();

      turnWithClawGoal(92, 1500);

      lowerTilter(100, -540, false);

      raiseLift();

      moveStraight(83, 5000);

      raiseTilterWithGoal();
      
      turnWith2Goals(0, 1500);

      lowerTilter();

      moveStraight(-26, 2000);

      raiseLiftFully();

      turnWithClawGoal(270, 1500);

      moveStraight(-12, 1200, 6.0);

      raiseLift(550, true);

      clawToggle();

      wait(400, msec);

      raiseLiftFully();

      moveStraight(6, 1000, 5.9);

      turnToAngle(185, 1900);

      lowerLift();

      moveStraight(-34, 2000);

      clawToggle();
      wait(200, msec);

      raiseLift(100, false);

      raiseTilter();

      turnWith2Goals(0, 2000);
      moveStraight(-6, 1000);
      lowerTilter();
      moveStraight(12, 1500);

      raiseTilterWithGoal();

      moveStraight(-46, 2000);
      raiseLiftFully();
      turnWith2Goals(270, 2000);

      moveStraight(-12, 1500, 6.0);

      clawToggle();

      wait(400, msec);

      moveStraight(10, 1000);

      turnWithTilterGoal(0, 1000);

      lowerLift();

      moveStraight(-30, 1200);

      turnWithTilterGoal(55, 1500);

      moveStraight(-15, 1500, 5.9);

      clawToggle();
      wait(200, msec);

      raiseLift();
      
      turnWith2Goals(90, 1500);

      moveStraight(-30, 1500);

      turnWith2Goals(135, 1500);

      lowerTilter(100, -540, false);

      raiseLiftFully();

      moveStraight(-30, 1500, 5.9);

      raiseTilter();

      clawToggle();
      wait(400, msec);

      moveStraight(20, 1500);

      turnToAngle(315, 1500);
      lowerLift();
      moveStraight(-10, 1000, 5.9);

      clawToggle();
      wait(200, msec);

      raiseLiftFully();

      turnWithClawGoal(135, 1500);

      moveStraight(-30, 1500);

      clawToggle();
      wait(400, msec);

      moveStraight(30, 1500);


      break;
    }
  }
}

brakeType lockDrive = coast;

void lockDriveHold() {
  lockDrive = hold;
}

void lockDriveCoast() {
  lockDrive = coast;
}

void usercontrol(void) {

  /*GYRO.startCalibration();
  while(GYRO.isCalibrating()) {
    wait(100, msec);
  }
  wait(2000, msec);*/

  Controller1.ButtonA.pressed(clawToggle); //little callback for toggling the claw pneumatics

  Controller1.ButtonX.pressed(lockDriveHold); //callbacks for toggling the locking drive code
  Controller1.ButtonY.pressed(lockDriveCoast);

  while (1) {

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
    } else if (Controller1.ButtonR2.pressing() && (!limLift.pressing())) {
      mArm.spin(reverse, 100, percent);
    } else {
      mArm.stop(hold);
    }

    //MOGO Tray
    if(Controller1.ButtonUp.pressing()) {
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
