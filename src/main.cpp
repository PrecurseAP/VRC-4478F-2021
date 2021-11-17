#include "vex.h"
#include "auton-movement.h"
#define TOGGLED_ON true
#define TOGGLED_OFF false
#include <string>
#include <iostream>
#include "pure-pursuit.h"
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// mLUpper              motor         12              
// mLLower              motor         11              
// mRUpper              motor         20              
// mRLower              motor         19              
// GPS                  gps           16              
// mConveyor            motor         18              
// mLTray               motor         1               
// mArm                 motor         13              
// clawPiston           digital_out   F               
// GYRO                 inertial      17              
// mRTray               motor         14              
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

int autonSelection = 0;

enum autonPath {
  rightAWP = 0,
  leftAWP = 1,
  rightNoAWP = 2,
  leftNoAWP = 3,
  fullAWP = 4,
  testing = 5
};

std::string autonSelections[6] = { "Right Side With AWP", "Left Side With AWP", 
                                "Right Side Without AWP", "Left Side Without AWP",
                                "fullAWP", "testing" };

void cycleAuton() {
  if (autonSelection+1 == 6) {
    autonSelection = 0;
  } else {
    autonSelection++;
  }
}

template <typename T> 
int sgn(T val) { //SIGNUM
    return (T(0) < val) - (val < T(0));
}
template <typename U>
U logDrive(U s) {
  return (s*s) / 100 * sgn(s);
}
template <typename S>
S logDriveVolt(S s) {
  return (s*s) / 12 * sgn(s);
}

competition Competition;

bool clawState = TOGGLED_OFF;

void clawToggle() {
  clawPiston.set(!clawState);
  clawState = !clawState;
}

void pre_auton() {
  vexcodeInit();
  Brain.Screen.print(autonSelection);
  GPS.startCalibration();
  GYRO.startCalibration();
  while(GPS.isCalibrating() || GYRO.isCalibrating()) {
    wait(100, msec);
  }
  wait(2500, msec);
  
}

void autonomous(void) {
  mArm.setPosition(0, degrees);
  mLTray.setPosition(0, degrees);
  mRTray.setPosition(0, degrees);
  //autonSelection=leftNoAWP;
  //move(-25, 100, 12, 3000);
  //wait(1000, msec);
  
  switch(autonSelection) {
    case rightAWP:
      raiseLift(false);

      lowerTilter(false);

      move(43, 100, 5, 2500);
      lowerLift(false);

      raiseTilterWithGoal(false);
      wait(250, msec);
      move(-20, 100, 12, 1400);

      spotTurnWithTilterGoal(180, 100, 8, 1700);

      move(22, 100, 13, 1500);

      lowerTilter(true);

      move(-10, 100, 10, 1500);

      spotTurn(90, 100, 12, 1500);

      move(15, 100, 15, 1000);

      raiseTilterWithGoal(false);
      wait(500, msec);
      spinConveyor();
      move(-23, 100, 14, 1500);
      mConveyor.stop(coast);

      lowerTilter(false);
      break;

    case leftAWP: 
      raiseLift(true);
      //spotTurn(9.5, 100, 7, 700);
      lowerLift(false);
      
      move(-49, 100, 7, 1900); 
      mArm.stop(hold);

      clawToggle();
      wait(300, msec);

      move(30, 100, 16, 2000);

      clawToggle();
      wait(300, msec);

      //spotTurn(6, 100, 10, 1400);

      moveSlow(16, 100, 15, 1500);

      spotTurn(267, 100, 13, 1250);

      //lowerTilter(false);
      
      moveSlow(-6, 100, 6, 900);
      lowerTilterWithValue(true, -550);
      moveSlow(12, 100, 6, 1200);
      lowerTilterWithValue(true, -270);

      moveSlow(-5, 100, 10, 1000);  
      spinConveyor();                                       //HIIIIIIII AIDEEEEEN!!!!!!!!!!!!!!!
      wait(900, msec);                                      //from connor (!)
      mConveyor.stop(coast);
      break;

    case rightNoAWP:
      raiseLift(false);
      lowerTilter(false);

      move(43, 100, 5, 2000);
      lowerLift(false);
      raiseTilterWithGoal(false);
      wait(250, msec);
      //spinConveyor();
      move(-27, 100, 12, 1500);
      //mConveyor.stop(coast);

      spotTurnWithTilterGoal(133, 100, 15, 2500);

      move(-38, 100, 12, 1500);

      clawToggle();
      wait(300, msec);

      spotTurnWith2Goals(133, 100, 12, 2000);
      move(56, 100, 10, 2800);

      lowerTilter(false);
      break;

    case leftNoAWP:
      raiseLift(false);
      //spotTurn(9.5, 100, 12, 800);
      //lowerLift(false);
      lowerTilter(false);


      move(46, 100, 4, 2500);  
      lowerLift(false);
      raiseTilterWithGoal(false);
      //clawToggle();
      wait(250, msec);

      //spinConveyor();
      move(-14, 100, 13, 1400);
      //raiseLift(true);
      //mConveyor.stop(coast);

      spotTurnWithNoAngleWrap(236, 100, 18, 4000);
      lowerTilter(false);
      wait(500, msec);
      move(-30, 100, 8, 1600);
      raiseTilter(false);
      //raiseTilterWithGoal(false);
      clawToggle();
      wait(300, msec);

      //move(28, 100, 10, 1400);

      spotTurnWith2Goals(213, 100, 12, 1500);

      move(50, 100, 12, 2000);

      break;
    case fullAWP:
      raiseLift(true);
      lowerLift(false);
      move(-45, 100, 1, 2200);
      clawToggle();
      wait(250, msec);

      raiseLift(false);

      move(28, 100, 5, 1600);

      spotTurnWithClawGoal(270, 100, 8, 1400);

      move(-5, 100, 5, 800);

      lowerTilter(true);

      move(11, 100, 4, 1000);

      raiseTilterWithGoal(true);

      spinConveyor();
      wait(300, msec);
      mConveyor.stop(hold);
      lowerTilterSlow(false);
      move(-98, 100, 5, 3300);
      
      raiseTilter(false);
      clawToggle();
      spotTurn(316, 100, 6, 1400);

      move(18, 100, 5, 1500);
      
      spinConveyor();
      wait(700, msec);
      mConveyor.stop(coast);


      break;
    case testing:
      clawToggle();
      wait(250, msec);
      raiseLiftFully(true);

      spotTurnWithClawGoal(96, 100, 45, 5000);
      lowerLift(false);
      lowerTilter(false);
      move(86, 100, 20, 3500);
      raiseTilterWithGoal(true);
      spotTurnWith2Goals(0, 100, 20, 3000);
      lowerTilter(true);
      move(-27, 100, 20, 2000);
      raiseTilter(false);
      raiseLiftFully(true);
      spotTurn(270, 100, 20, 3000);
      moveSlow(-5.5, 100, 20, 1500);
      clawToggle();
      wait(1000, msec);
      moveSlow(3, 100, 20, 1500);
      spotTurn(176, 100, 20, 3000);
      lowerLift(false);
      move(40, 100, 20, 3000);
      spotTurn(245, 100, 20, 3000); //turn to blue goal
      moveSlow(-11.3, 100, 20, 3000);
      clawToggle();
      wait(300, msec);
      moveSlow(8, 100, 20, 3000);
      spotTurnWithClawGoal(269, 100, 25, 4000);
      lowerTilter(false);
      wait(300, msec);
      move(46, 100, 20, 3000);
      raiseTilterWithGoal(true); //lift right yellow moog
      move(26, 100, 20, 3000);
      spotTurnWith2Goals(180, 100, 20, 3500);
      lowerTilter(true);
      move(-27, 100, 20, 3000);
      raiseLiftFully(true);
      spotTurnWithClawGoal(90, 100, 20, 3000);
      moveSlow(-8.5, 100, 20, 3000);
      clawToggle();
      wait(300, msec);
      spotTurn(90, 100, 80, 6000);
      //turnToPoint(0, 0, 30, 5000);
      moveSlow(35, 100, 20, 3000);
      lowerLift(false);
      raiseTilterWithGoal(true);
      spotTurnWithTilterGoal(129, 100, 20, 3000);
      move(40, 100, 20, 3000);
     /* clawToggle();
      wait(300, msec);
      spotTurnWith2Goals(90, 100, 20, 3000);
      move(65, 100, 20, 3000);*/



      break;
    default:
      break;
  
  }
}

void usercontrol(void) {
  //drawOnBrain(inject(path1, 10), color::white);   
 drawOnBrain(path1,color::white, 4);
  drawOnBrain(smoother(inject(path1, 15), .9, 1), color::red, 2);
  
  wait(1000000, msec);
  Controller1.ButtonA.pressed(clawToggle);
  bool LockDrive=false;
  while(1) {

    float LSpeed = logDriveVolt(Controller1.Axis3.position(percent)*(float)(12.0/100.0));
    float RSpeed = logDriveVolt(Controller1.Axis2.position(percent)*(float)(12.0/100.0));

    //std::cout << LSpeed << std::endl;

    mLUpper.spin(forward, LSpeed, volt);
    mLLower.spin(forward, LSpeed, volt);
    mRUpper.spin(forward, RSpeed, volt);
    mRLower.spin(forward, RSpeed, volt);

    
    if ((LSpeed == 0)&&(LockDrive)){
      mLLower.stop(hold);
      mLUpper.stop(hold);
    }
    else if (LSpeed == 0){
      mLLower.stop(coast);
      mLUpper.stop(coast);
    }

    if((RSpeed == 0)&&(LockDrive)){
      mRLower.stop(hold);
      mRUpper.stop(hold);
    }
    else if (RSpeed == 0){
      mRLower.stop(coast);
      mRUpper.stop(coast);
    }


    //DriveLock
    if(Controller1.ButtonX.pressing())
    {
      LockDrive=true;
    }
    else if(Controller1.ButtonY.pressing())
    {
      LockDrive = false;
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
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while(true) {
    wait(100, msec);
  }
}
