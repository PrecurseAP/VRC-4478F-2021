//code written by Aiden Pringle :3

#include "vex.h"

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
// twLeft               rotation      15              
// twRight              rotation      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "utils.h"
#include "movements.h"
#include "autonomous.h"
using namespace vex;

competition Competition;

template <typename T> 
int sgn(T val) {
  //signum function: returns the sign of the input
  //x < 0: -1
  //x = 0: 0
  //x > 0: 1
  return (T(0) < val) - (val < T(0));
}

template <typename S>
S logDriveVolt(S s) {
  //logarithmic drive function, written for voltage.
  return pow(12, 2.6) / pow(12, 1.6) * sgn(s);
}

int autonSelection = 0;
std::string autonPath = "Right 20P + rings";

void cycleAuton() {
  //auton choice cycling function
  //loops a variable from 0 to a number each time the screen is tapped.
  //Number depends on the amount of auton paths, each number 0 to n corresponds to a route

  if (autonSelection < 7) {
    autonSelection++;
  } else {
    autonSelection = 0;
  }

  //names for each route, these are drawn on the screen so we know what we are choosing
  switch (autonSelection) {
    case 0: {
      autonPath = "RIGHT 20p + rings";
      break;
    }
    case 1: {
      autonPath = "LEFT 20p + RINGS";
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
      autonPath = "SOLOAWP";
      break;
    }
    case 5: {
      autonPath = "right side center sprint";
      break;
    }
    case 6: {
      autonPath = "right ally then neutral";
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

  //set up for the route selection
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(autonPath.c_str());
  Brain.Screen.render();

  Brain.Screen.pressed(cycleAuton);

  //calibrate the inertial sensor
  GYRO.startCalibration();
  while(GYRO.isCalibrating()) {
    wait(100, msec);
  }
  wait(2500, msec);

}

void autonomous(void) {

  //The autonomous function.
  //The selected path carries over from the preauton function and is used in the switch statement.
  //Each number has a corresponding path, this is where the path is then run.

  //reset ALL motors to default positions
  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mBackLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);
  mLift.setPosition(0, degrees);
  mConveyor.setPosition(0, degrees);

  autonSelection = 7;

  switch(autonSelection) {
    //each path prints to the vexcode console, in ms, how much time the route took to finish.
    case 0: /*right 20*/ {
      int t = rightBasic20Rings();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
    case 1: /*left 20*/ {
      int t = leftBasic20Rings();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
    case 2: /*right 40 awp*/ {
      int t = right40AWP();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
    case 3: /*left center dash*/ {
      int t = leftCenterDash();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
    case 4: /* full awp*/ {
      int t = soloAWP();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
    case 5: /* right center dash + rings*/ {
      int t = rightCenterDash();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
    case 6: /*Right ally then neutral*/{
      int t = rightNeutralLast();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
    case 7: /* skills */ {
      int t = runSkills();
      std::cout << "Auton completed in " << t << " ms." << std::endl;
      break;
    }
  }

}

bool tilterToggled = true;

void toggleTilter() {
  tilterPiston.set(!tilterToggled);
  tilterToggled = !tilterToggled;
}

bool clawToggled = true;

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
  //User control function. Runs during driver control. (duh)

  //testing for turning functions ignore these

  //set up callbacks for piston toggles and braketype toggles.
  Controller1.ButtonR2.pressed(toggleTilter);
  Controller1.ButtonR1.pressed(toggleClaw);

  Controller2.ButtonA.pressed(toggleTilter);

  Controller1.ButtonX.pressed(lockDriveHold); 
  Controller1.ButtonY.pressed(lockDriveCoast);

  while (1) {

    //get logarithmic drive speed, scale percentage to voltage
    float LSpeed = logDriveVolt(Controller1.Axis3.position(percent)*(float)(12.0/100.0));
    float RSpeed = logDriveVolt(Controller1.Axis2.position(percent)*(float)(12.0/100.0));

    //set motors to respective speeds
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

    //Lockdrive, basically we toggle between hold and coast based on situation. (parking vs defense etc)
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

    //run lift and conveyor based on controller inputs
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
  //Main function. I never touch this, only sets up competition template.

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
