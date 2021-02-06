#include "vex.h"
#include "robot-config.h"
#include "odom.h"
#include "drive.h"
#include "pre-auton.h"
#include "drive.h"

using namespace vex;

competition Competition;

void pre_auton(void) {
  renderScreen(); //draw the field on the screen once.
  Brain.Screen.pressed(touchScreenLogic); //callback so that the drawing and logic code is only executed when the screen is touched. (this saves tons of resources as opposed to a loop)
}

void autonomous(void) {
  //the moveToPoint function accepts (x, y, angle, xMult, yMult, tMult)
  //All of the arguments are measured in inches for distance, degrees for angles.

  thread ODOM = thread(tracking); //start position tracking loop

  resetPos(); //reset variables / initialize the values

  moveToPoint(23, 6.7, 180, 7, 6, 1); //move in front of the middle home goal

  driveStraightNoTracking(60); //inch forward so the ball makes it

  wait(200, msec);
  
  stopAllDrive(hold); //stop drive to shoot
  
  spinRollers(100); //shoot the ball into the middle goal
  
  wait(850, msec);
  
  stopRollers(hold); //stop the rollers
  
  spinIntakes(100); //start intaking to flip them out and collect next balls

  moveToPoint(-21, 5, 226, 4, 4, 1); //move in front of left home row goal

  driveStraightNoTracking(27); //inch forward to collect ball and approach toward

  wait(700, msec);

  spinRollers(100); //start shooting

  wait(300, msec);  
  
  stopIntakes(hold); //stop intakes to not empty tower
  
  stopAllDrive(hold); //stop drive to shoot

  wait(1100, msec);
  
  stopRollers(hold);

  driveStraightNoTracking(-100); //zoom backwards so next move doesnt collide with tower
  
  wait(1100, msec);
  
  moveToPoint(64, 14, 135 , 3, 3, 1); //move in front of right home row tower
  
  spinIntakes(100); //start intaking
  
  driveStraightNoTracking(30); //inch forward to reacht he tower and collect ball
  
  wait(300, msec);
  
  spinRollers(100); //start shooting
  
  wait(1100, msec);
  
  stopIntakes(hold); //stop intakes to not empty tower
  
  stopAllDrive(hold); //stop drive to shoot
  //gimme three ball
}

void usercontrol(void) {
  ODOM.thread::interrupt();
  Controller1.ButtonX.pressed(resetGyro);
  //user control 
  driveTheDamnRobot();
}

int main() {
  vexcodeInit();
  
  resetGyro();
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) { wait(100, msec); }
} 