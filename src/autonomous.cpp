#include "vex.h"
#include "utils.h"
#include "autonomous.h"
#include "movements.h"

int rightBasic20Rings() {      
  int t = Brain.timer(msec);

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

  return Brain.timer(msec) - t;
}

int leftBasic20Rings() {
  int t = Brain.timer(msec);      
      
  clawPiston.set(true);
  tilterPiston.set(true);
  moveStraight(46, 1250, 5.80);
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

  return Brain.timer(msec) - t;
}

int right40AWP() {
  int t = Brain.timer(msec);
     
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
  
  return Brain.timer(msec) - t;
}

int leftCenterDash() {
  int t = Brain.timer(msec);

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

  return Brain.timer(msec) - t;
}

int soloAWP() {
  int t = Brain.timer(msec);

  //grip da ally goal and ring it
  clawPiston.set(true);
  tilterPiston.set(false);
  mConveyor.spin(forward, 450, rpm);

  //move up a bit and turn to midfield
  moveStraight(5, 500);
  turnWithTilterGoal(270, 1500);
  moveStraight(-30, 1000);

  //turn to +'s of rings and deposit them (hopefully)
  turnWithTilterGoal(180, 1200);
  basicDrive(35);
  wait(2000, msec);
  stopAllDrive(hold);

  //let go of goal and turn to ally goal, move to it
  tilterPiston.set(true);
  turnToAngle(0, 1000);
  moveStraight(40, 1500, 5);

  //grab dat goal and start ringing it
  tilterPiston.set(false);
  wait(50, msec);
  moveStraight(10, 750);
  turnWithTilterGoal(90, 1200);

  /*
  old route
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
  mConveyor.stop(hold);*/

  return Brain.timer(msec) - t;
}

int rightCenterDash() {
  int t = Brain.timer(msec);

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

  return Brain.timer(msec) - t;
}

int runSkills() {
  int t = Brain.timer(msec);
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

  return Brain.timer(msec) - t;
}