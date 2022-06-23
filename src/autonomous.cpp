#include "vex.h"
#include "utils.h"
#include "autonomous.h"
#include "movements.h"

//main source file for autonomous routes.

//All functions capture start and end times and return them. These are printed to the vexcode console.

//dashes to the rightmost neutral goal, backs up, grabs alliance goal, and puts field rings onto it.
int rightBasic20Rings() {      
  int t = Brain.timer(msec);

  tilterPiston.set(true);
  clawPiston.set(true);

  //move to neutral goal, grab it, move back
  moveStraight(44, 1200, 5.9);
  clawPiston.set(false);
  wait(75, msec);
  moveStraight(-30, 15000);

  //turn to alliance goal, grab it in tilter
  raiseLift(100, true);
  turnWithClawGoal(270, 1500);
  clawPiston.set(true);
  wait(50, msec);
  moveStraight(-11, 1500);
  moveStraight(-3, 500, 5.50);
  tilterPiston.set(false);
    
  //raise lift so rings have clearance, align bot with ring line on field
  raiseLift(200, true);
  moveStraight(9, 1000);
  turnWithTilterGoal(0, 1500);

  //begin suckage of rings onto the alliance goal. Turn to other line of rings against wall
  mConveyor.spin(forward, 600, percent);
  basicDrive(40);
  wait(1400, msec);
  turnWithTilterGoal(90, 1000);
  basicDrive(45);
  wait(600, msec);
  stopAllDrive(hold);
  moveStraight(-12, 600);
  
  //turn back to home and run away
  turnWithTilterGoal(0, 1000);
  moveStraight(-35, 1500);

  tilterPiston.set(true);

  moveStraight(10, 1500);

  return Brain.timer(msec) - t;
}

//left auton that grabs leftmost neutral goal
int leftBasic20Rings() {
  int t = Brain.timer(msec);
  clawPiston.set(true);
  tilterPiston.set(true);
  
  //dash to neut goal, pick it up, move back
  moveStraight(45.6, 1250, 5.80);
  moveStraight(1, 250);
  clawPiston.set(false);
  wait(50, msec);
  moveStraight(-40, 15000);

  //turn to alliance goal against platform, run up to it, shoot preloads. No grabbing, too inconsistent
  clawPiston.set(true);
  turnToAngle(280, 1500);
  moveStraight(-11, 1500);
  wait(300, msec);
  tilterPiston.set(false);
  /*wait(100, msec);
  moveStraight(12, 700);*/
  mConveyor.spin(forward, 550, rpm);
  wait(1500, msec);
  mConveyor.stop();
  moveStraight(15, 1500);
  tilterPiston.set(true);
  moveStraight(4, 500);
  moveStraight(-8, 800);
  

  //tilterPiston.set(true);
  

  return Brain.timer(msec) - t;
}
int left40Point() {
  int t = Brain.timer(msec);

  clawPiston.set(true);
  tilterPiston.set(true);
  

  followArc(65, 40, 1500, 200, 1, false);
  wait(75, msec);
  clawPiston.set(false);
  wait(50, msec);
  moveStraight(-30, 1500);
  raiseLift(180, false);
  odomTurn(60, 30, 1500);

  mConveyor.spin(forward, 90, percent);
  moveStraight(55, 2000);
  mConveyor.stop(hold);
  odomTurn(50, -70, 2000);
  moveStraight(-12, 1500, 5);
  wait(100, msec);
  tilterPiston.set(false);
  odomTurn(-10, 0, 1500); 
  moveStraight(65, 2500);

  tilterPiston.set(true);
  
  odomTurn(-100, 35, 1000);

  moveStraight(-15, 1500, 5.5);
  wait(50, msec);
  tilterPiston.set(false);
  mConveyor.spin(forward, 100, percent);
  turnToAngle(270, 1000);
  moveStraight(20, 2500, 5.5, 30);

  return Brain.timer(msec) - t;
}
int right40AWP() {
  int t = Brain.timer(msec);
     
  tilterPiston.set(true);
  clawPiston.set(true);

  //run up to rightmost neut goal, 
  moveStraight(45, 1200, 5.9);
  clawPiston.set(false);
  wait(75, msec);
  raiseLift(60, false);
  moveStraight(-34, 1500);

  turnWithClawGoal(138, 1500);
  clawPiston.set(true);
  raiseLift(0, false);
  moveStraight(-20, 1000);
  turnToAngle(319, 1000);
  moveStraight(19, 750, 5.2);
  clawPiston.set(false);
  raiseLift(60, false);
  moveStraight(-41, 1200);
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

  tilterPiston.set(true);
  
  return Brain.timer(msec) - t;
}

int leftCenterDash() {
  int t = Brain.timer(msec);

  clawPiston.set(true);
  tilterPiston.set(true);
  moveStraight(38, 1500, 5.82);
  turnToAngle(270, 1500);
  moveStraight(-58, 2500);
  moveStraight(6, 800);
  turnToAngle(0, 1500);
  moveStraight(10, 1500, 5.50, 40);
  clawPiston.set(false);
  wait(200, msec);
  raiseLift(100, false);
  turnWithClawGoal(45, 1500);
  moveStraight(-66, 1500);
      
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

  tilterPiston.set(true);

  return Brain.timer(msec) - t;
}

int soloAWP() {
  int t = Brain.timer(msec);

  //grip da ally goal and ring it
  clawPiston.set(true);
  tilterPiston.set(true);
  wait(150, msec);
  moveStraight(-4, 500);
  tilterPiston.set(false);
  mConveyor.spin(forward, 500, rpm);

  //move up a bit and turn to midfield
  moveStraight(5, 500);
  turnWithTilterGoal(268, 1300);
  moveStraight(-32, 900);

  //turn to +'s of rings and deposit them (hopefully)
  raiseLift(150, false);
  turnWithTilterGoal(180, 1000);
  basicDrive(45);
  wait(2810, msec);
  stopAllDrive(hold);

  //let go of goal and turn to ally goal, move to it
  tilterPiston.set(true);
  mConveyor.stop(hold);
  turnToAngle(19, 1000);
  mConveyor.spin(reverse, 110, rpm);
  moveStraight(-28, 1000, 5.2);

  //grab dat goal and start ringing it
  tilterPiston.set(false);
  wait(50, msec);
  mConveyor.spin(forward, 500, rpm);
  moveStraight(10, 750);
  turnWithTilterGoal(90, 1000);
  basicDrive(50);
  wait(1090, msec);
  stopAllDrive(hold);
  turnWithTilterGoal(180, 1000);
  basicDrive(45);
  wait(500, msec);
  stopAllDrive(hold);
  moveStraight(-8, 600);
  
  //turn back to home and run away
  turnWithTilterGoal(135, 1000);
  moveStraight(-30, 1500);

  tilterPiston.set(true);

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

  //move up, then turn to the center goal
  moveStraight(17.5, 1000);
  turnToAngle(313, 1000);

  //move to it, then grab it and run back
  moveStraight(39.5, 1500, 5.5);
  clawPiston.set(false);
  wait(100, msec);
  raiseLift(50, true);
  moveStraight(-41, 1500);

  //drop tall goal in home zone, move up to alliance goal
  turnWithClawGoal(270, 1500);
  clawPiston.set(true);
  wait(50, msec);
  moveStraight(-18, 1500);
  moveStraight(-3, 500);

  //grab alliance goal, move a back a bit
  tilterPiston.set(false);
  moveStraight(11, 1000);
  turnWithTilterGoal(0, 1500);

  //raise 
  raiseLift(200, true);
  mConveyor.spin(forward, 600, percent);
  basicDrive(30);
  wait(1700, msec);
  moveStraight(-35, 1500);

  tilterPiston.set(true);

  return Brain.timer(msec) - t;
}

int rightNeutralLast() {
  int t = Brain.timer(msec);

  tilterPiston.set(true);
  clawPiston.set(true);

  //move up to alliance goal, grab it in tilter
  moveStraight(-8, 1000);
  moveStraight(-3, 400);
  tilterPiston.set(false);
  wait(100, msec);
  mConveyor.spin(forward, 500, rpm);

  //move over to neutral goal, grab it in claw
  turnWithTilterGoal(270, 1000);
  moveStraight(-21, 1000);
  mConveyor.stop();
  turnWithTilterGoal(180, 1100);
  moveStraight(33, 1500, 5.3);
  moveStraight(2, 250);
  clawPiston.set(false);

  //move back to edge of field and get ready for driver loads
  moveStraight(-35, 1500);
  turnWith2Goals(0, 1400);
  moveStraight(-17, 1000);
  raiseLift(475, true);
  wait(1400, msec);
  mConveyor.spin(forward, 550, rpm);

  //pick a line of driver loads.
  basicDrive(30);
  wait(2000, msec);
  basicDrive(-10);
  wait(1400, msec);
  raiseLift(60, true);
  /*repeat(5) {
    basicDrive(30);
    wait(700, msec);
    basicDrive(-30);
    wait(300, msec);
  }*/// made u look
  stopAllDrive(hold);

  tilterPiston.set(true);

  return Brain.timer(msec) - t;
}

int runSkills() {
  int t = Brain.timer(msec);
  
  clawPiston.set(true);
  tilterPiston.set(true);

  //grab first allied goal
  wait(150, msec);
  moveStraight(-4, 500);
  tilterPiston.set(false);

  //jump back, turn and move to neutral goal
  moveStraight(5, 500);
  turnWithTilterGoal(271, 1300);
  moveStraight(-30, 1100);

  //adjust ange so that we can grab yellow, move into it
  turnWithTilterGoal(107, 1300);
  moveStraight(15, 1400, 6, 40);
  moveStraight(3, 300);
  wait(100, msec);

  //grab the goal, move back some
  clawPiston.set(false);
  wait(50, msec);
  moveStraight(-25, 1000);
  raiseLift(150, true);
  
  //turn into the ring stars, begin intaking them
  turnWith2Goals(183, 1400);
  mConveyor.spin(forward, 500, rpm);
  moveStraight(45, 4500, 6, 35);

  //turn towards platform, then drop neutral goal
  turnWith2Goals(270, 1200);
  raiseLift(425, true);
  moveStraight(17, 2500, 6, 50);
  clawPiston.set(true);

  //unclog conveyor, move to the right of the field
  mConveyor.spin(reverse, 75, rpm);
  moveStraight(-10, 1000);
  raiseLift(120, false);
  turnWithTilterGoal(180, 1500);
  mConveyor.spin(forward, 500, rpm);
  moveStraight(33, 1800);
  

  //turn to the next neutral goal, go and grab it
  turnWithTilterGoal(89, 1000);
  raiseLift(0, false);
  moveStraight(12, 1400, 6, 50);
  moveStraight(3, 300, 6, 50);
  wait(100, msec);
  clawPiston.set(false);
  wait(50, msec);

  //raise lift so no friction, then move back and turn towards the platform
  mConveyor.spin(reverse, 75, rpm);
  raiseLift(90, true);
  moveStraight(-20, 1000);
  mConveyor.spin(forward, 500, rpm);
  turnWith2Goals(2, 1500);
  moveStraight(35, 1500);

  //raise the lift, then put the goal on the platform.
  raiseLift(425, true);
  turnWith2Goals(270, 1500);
  moveStraight(23, 2500, 6, 65);
  clawPiston.set(true);
  wait(50, msec);

  //move back a bit, turn to center goal, grab it 
  moveStraight(-18, 1000);
  raiseLift(0, false);
  turnWithTilterGoal(89, 1500);
  moveStraight(16, 1800, 6, 40);
  moveStraight(2, 200);
  clawPiston.set(false);

  //take the center goal to the right corner of the field
  wait(100, msec);
  raiseLift(75, true);
  turnWith2Goals(232, 1500);
  moveStraight(67, 2500);
  raiseLift(0, true);
  clawPiston.set(true);

  //move back, turn to alliance goal and grab it
  wait(50, msec);
  moveStraight(-9, 1000);
  turnWithTilterGoal(182, 1100);
  moveStraight(18, 1500, 6, 70);
  moveStraight(3, 600);
  clawPiston.set(false);
  wait(50, msec);
  moveStraight(-5, 600);
  raiseLift(150, true);

  //move back a bit, turn towards opposite platform, run to it.
  moveStraight(-5, 700);
  raiseLift(150, true);
  turnWith2Goals(61, 1500);
  mConveyor.spin(forward, 600, rpm);
  moveStraight(72, 3500);

  //raise lift, then try to level the platform and put the mobile onto it.
  raiseLift(450, true);
  moveStraight(18, 1500, 6, 50);
  raiseLift(375, true);
  raiseLift(420, true);
  moveStraight(-5, 1500, 6, 50);
  turnWith2Goals(95, 1500);
  moveStraight(8, 1500, 6, 40);
  clawPiston.set(true);

  //move back and try to pick up the goal in our tilter, then elevate it
  moveStraight(-20, 1500);
  raiseLift(0, true);
  moveStraight(5, 600);
  tilterPiston.set(true);
  wait(200, msec);
  moveStraight(9, 1000);
  turnToAngle(273, 1500);
  moveStraight(17, 1500, 6, 60);
  clawPiston.set(false);
  raiseLift(400, true);
  turnWithClawGoal(95, 1500);
  moveStraight(32, 2000, 6, 50);
  clawPiston.set(true);
  moveStraight(-10, 1500);

  //turn to red ally goal and grab it hopefully
  turnToAngle(11, 1500);
  raiseLift(0, false);
  moveStraight(56, 2000);
  moveStraight(3, 300);
  clawPiston.set(false);
  wait(100, msec);
  moveStraight(-10, 1000);
  turnWithClawGoal(270, 1000);
  moveStraight(70, 2500);
  


  return Brain.timer(msec) - t;
}

int leftAllyFirst() {
  int t = Brain.timer(msec);
  
  clawPiston.set(true);
  tilterPiston.set(true);

  //grab first allied goal
  wait(150, msec);
  moveStraight(-4, 500);
  tilterPiston.set(false);

  //jump back, turn and move to neutral goal
  moveStraight(5, 500);
  turnWithTilterGoal(271, 1300);
  mConveyor.spin(forward, 500, rpm);
  moveStraight(-30, 1100);

  //adjust ange so that we can grab yellow, move into it
  turnWithTilterGoal(115, 1300);
  mConveyor.spin(reverse, 150, rpm);
  moveStraight(17, 1400, 6, 40);
  wait(100, msec);

  //grab the goal, move back some
  clawPiston.set(false);
  wait(50, msec);
  moveStraight(-15, 1000);
  raiseLift(450, false);

  //turn back to field wall
  turnToAngle(270, 2000);
  moveStraight(14, 1500);
  mConveyor.spin(forward, 500, rpm);
  moveStraight(40, 2500, 6, 40);
  moveStraight(-15, 2000);

  tilterPiston.set(true);

  return Brain.timer(msec) - t;

}
