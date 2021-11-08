#include "vex.h"
#include "odometry.h"
#define TOGGLED_ON true
#define TOGGLED_OFF false
#include <string>
#include <iostream>
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
  fullAWP = 4
};

std::string autonSelections[5] = { "Right Side With AWP", "Left Side With AWP", 
                                "Right Side Without AWP", "Left Side Without AWP",
                                "fullAWP" };

void cycleAuton() {
  if (autonSelection+1 == 5) {
    autonSelection = 0;
  } else {
    autonSelection++;
  }
}

void renderScreen() {
  cycleAuton();
  Brain.Screen.print(autonSelection);

  /*
  //for reference the screen is 480 x 272 pixels
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor("#777777"); //change border color of squares to a slightly darker gray, separates tiles
    
  for (int ROW = 0; ROW < 6; ROW++) 
    for (int COL = 0; COL < 6; COL++) 
      Brain.Screen.drawRectangle( (COL * 35) + 133 , (ROW * 35) + 18 , 35 , 35 , "#888888");
  //^^ The above nested loop generates the 6x6 grid of field tiles in gray

  switch(p.touchedSquare) {
    case leftRed:
      Brain.Screen.drawRectangle(133, 53, 35, 35, color::white); break;
    case rightRed:
      Brain.Screen.drawRectangle(133, 158, 35, 35, color::white); break;                                                                        
    case leftBlue:                                                          //logic to highlight the square that was pressed
      Brain.Screen.drawRectangle(308, 158, 35, 35, color::white); break;
    case rightBlue:
      Brain.Screen.drawRectangle(308, 53, 35, 35, color::white); break;
    default: break;
      //do nothing if there is no chosen square
  }

  Brain.Screen.setPenColor(color::red);
  Brain.Screen.drawRectangle(113, 20, 3, 205, color::red);   //draw leftmost red line

  Brain.Screen.setPenColor(color::blue);
  Brain.Screen.drawRectangle(363, 20, 3, 205, color::blue);  //draw rightmost blue line 

  Brain.Screen.setPenColor(color::white); //change to white

  Brain.Screen.drawRectangle(133, 120, 35, 2);               //draw two horizontal white lines on the left
  Brain.Screen.drawRectangle(133, 124, 35, 2);               //
                                                
  Brain.Screen.drawRectangle(167, 18, 2, 209);               //draw vertical white line on the left

  Brain.Screen.drawRectangle(235, 18, 2, 209);               //draw two vertical white lines in the center                                  
  Brain.Screen.drawRectangle(239, 18, 2, 209);               //

  Brain.Screen.drawRectangle(307, 18, 2, 209);               //draw vertical white line on the right

  Brain.Screen.drawRectangle(308, 120, 34, 2);               //draw two horizontal white lines on the right                                                                                    
  Brain.Screen.drawRectangle(308, 124, 34, 2);               //

  Brain.Screen.setPenColor("#444444"); //change color to a dark gray

  Brain.Screen.drawRectangle(133, 18, 209, 2); //
  Brain.Screen.drawRectangle(133, 226, 209, 2);//     Draw field border walls in dark gray
  Brain.Screen.drawRectangle(133, 18, 2, 209); //                                                
  Brain.Screen.drawRectangle(341, 18, 2, 209); //

  Brain.Screen.setPenColor("#222222"); //set color to a very dark gray, pretty much black

  drawGoal(144, 29, color::blue);              //                
  drawGoal(238, 29, color::red);               //draw top three goals
  drawGoal(331, 29, color::red);               //

  drawGoal(144, 123, color::blue);             // 
  drawGoal(238, 123, color::transparent);      //draw middle three goals
  drawGoal(331, 123, color::red);              //

  drawGoal(144, 216, color::blue);             // 
  drawGoal(238, 216, color::blue);             //draw bottom three goals
  drawGoal(331, 216, color::red);              //

  Brain.Screen.drawCircle(155, 40, 6, color::red);
  Brain.Screen.drawCircle(320, 40, 6, color::blue);
  Brain.Screen.drawCircle(320, 205, 6, color::blue);
  Brain.Screen.drawCircle(155, 205, 6, color::red);
  Brain.Screen.drawCircle(238, 70, 6, color::red);          //DRAW ALL THE DAMN BALLS )that arent in goals(
  Brain.Screen.drawCircle(238, 175, 6, color::blue);
  Brain.Screen.drawCircle(238, 107, 6, color::red);
  Brain.Screen.drawCircle(238, 139, 6, color::blue);
  Brain.Screen.drawCircle(222, 123, 6, color::blue);
  Brain.Screen.drawCircle(254, 123, 6, color::red);

  if (p.autSel == false) { 
    Brain.Screen.drawRectangle(0, 0, 80, 272, "#007F00");       //Draw left and right confirmation buttons
    Brain.Screen.drawRectangle(400, 0, 80, 272, "#007F00");
  }
  else {
    Brain.Screen.drawRectangle(0, 0, 80, 272, "#00007F");       //Draw left and right unconfirmation buttons
    Brain.Screen.drawRectangle(400, 0, 80, 272, "#00007F");
  }
  Brain.Screen.printAt(4, 17, false, "Confirm");       //
  Brain.Screen.printAt(4, 234, false, "Confirm");      //     Draw confirm text on confirm buttons
  Brain.Screen.printAt(405, 17, false, "Confirm");     //
  Brain.Screen.printAt(405, 234, false, "Confirm");    //
  */
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
  Brain.Screen.pressed(renderScreen);
  //GPS.startCalibration();
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
  //autonSelection=leftAWP;
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


      move(49, 100, 4, 2500);  
      lowerLift(false);
      raiseTilterWithGoal(false);
      //clawToggle();
      wait(250, msec);

      //spinConveyor();
      move(-21, 100, 13, 1400);
      //raiseLift(true);
      //mConveyor.stop(coast);

      spotTurnWithNoAngleWrap(226, 100, 18, 3500);
      lowerTilter(false);
      wait(500, msec);
      move(-40, 100, 8, 1600);
      raiseTilter(false);
      //raiseTilterWithGoal(false);
      clawToggle();
      wait(300, msec);

      //move(28, 100, 10, 1400);

      //spotTurn(213, 100, 12, 1500);

      move(65, 100, 12, 2000);

      break;
    case fullAWP:
      move(-43, 100, 1, 2200);
      
      break;
    default:
      break;
  
  }
}

void usercontrol(void) {
  
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
