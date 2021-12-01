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
// mRTray               motor         14              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include "auton-movement.h"
#define TOGGLED_ON true
#define TOGGLED_OFF false
#include <string>
#include <iostream>
#include "pure-pursuit.h"

using namespace vex;

int deploy() {
  raiseLift(true);
  lowerLift(false);

  return 0; 
}

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
  Brain.Screen.print(autonSelection);
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

  Brain.Screen.pressed(cycleAuton);
  Brain.Screen.print(autonSelection);
  GPS.startCalibration();
  GYRO.startCalibration();
  while(GPS.isCalibrating() || GYRO.isCalibrating()) {
    wait(100, msec);
  }
  wait(2500, msec);
  
}

thread _deploy;

void autonomous(void) {
  mArm.setPosition(0, degrees);
  mLTray.setPosition(0, degrees);
  mRTray.setPosition(0, degrees);
  autonSelection=rightAWP;
  //move(-25, 100, 12, 3000);
  //wait(1000, msec);
  
  switch(autonSelection) {
    case rightAWP: { // big goals
      thread _deploy = thread(deploy);

      move(-47, 100, 4, 1500);
      clawToggle();
      wait(250, msec);
      //lowerTilter(false);
      raiseLift(false);
      moveSlow(29, 100, 7, 1300);

      spotTurnWithClawGoal(137.5, 100, 25, 1700);
      moveSlow(-15, 100, 5, 700);
      //lowerLift(true);
      clawToggle();
      wait(250, msec);
      lowerLift(false);
      move(35, 100, 10, 1600);
      spotTurn(327, 100, 2, 1200);
      moveSlow(-14, 100, 2, 1000);
      clawToggle();
      wait(250, msec);
      raiseLift(true);
      move(48, 100, 12, 1900);

      //lowerTilter(false);
      lowerTilter(false);
      spotTurnWithClawGoal(265, 100, 10, 1200);
      clawToggle();

      moveSlow(13, 100, 10, 800);

      raiseTilterWithGoal(true);

      spinConveyor();
      move(-30, 100, 2, 1000);
      mConveyor.stop(coast);
      break;
    }
    case leftAWP: {
      thread _deploy = thread(deploy);

      move(-49, 100, 2, 1500);
      clawToggle();
      wait(250, msec);
      move(33, 100, 20, 2500);
      raiseLift(true);
      spotTurnWithClawGoal(340, 100, 30, 3000);
      moveSlow(8, 100, 20, 1500);
      spinConveyor();
      wait(1500, msec);
      mConveyor.stop(coast);

      break;
    }
    case rightNoAWP: { //big rinigs
      thread _deploy = thread(deploy);

      move(-46, 100, 2, 1500);
      clawToggle();
      wait(250, msec);
      move(27, 100, 10, 1500);
      spotTurnWithClawGoal(274, 100, 15, 1500);
      clawToggle();
      wait(300, msec);
      moveSlow(-10, 100, 15, 1000);
      lowerTilter(true);
      moveSlow(18, 100, 15, 1000);
      raiseTilterWithGoal(true);
      spinConveyor();
      moveSlow(-10, 100, 12, 900);
      spotTurnWithTilterGoal(180, 100, 20, 1500);
      moveRightSide(40);
      moveLeftSide(40);
      wait(2000, msec);
      mConveyor.stop(coast);
      move(-50, 100, 2, 2000);

      break;
    }
    case leftNoAWP: { //40
      thread _deploy = thread(deploy);

      move(-47, 100, 4, 1500);
      clawToggle();
      wait(250, msec);
      raiseLift(false);

      move(22, 90, 4, 1200);
      lowerTilter(false);
      spotTurn(219, 100, 17, 1800);
      lowerLift(true);
      clawToggle();
      wait(250, msec);
      moveSlow(20, 100, 4, 1000);
      spotTurn(40, 100, 7, 1100);
      raiseTilter(false);
      moveSlow(-12, 100, 4, 800);
      clawToggle();
      wait(500, msec);
      raiseLift(false);
      move(54, 100, 4, 2000);
      clawToggle();
      wait(100, msec);
      spotTurn(322, 100, 4, 1000);
      moveSlow(9, 100, 4, 1500);
      spinConveyor();
      wait(1000, msec);
      mConveyor.stop();


      break;
    }
    case fullAWP: {
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
    }
    case testing: {//skills 
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
    }
    default: {
      break;
    }
  }
}

void usercontrol(void) {
  
  std::vector<Point> path2 = {
    Point(24, -24),    
    Point(24, 24),
    Point(-24, 24),
    Point(-24, -24)
  };

  std::vector<Point> autoPath = inject(path2, 4);
  autoPath = smoother(autoPath, .90, .1);
  autoPath = calculateDistances(autoPath);
  autoPath = calculateCurvatures(autoPath);
  autoPath = calculateVelocities(autoPath, 10, .4, 2, .75);

  //wait(3000, msec);

  //int a = purePursuit(autoPath, 11);

  Controller1.ButtonA.pressed(clawToggle); //little callback for toggling the claw pneumatics
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
