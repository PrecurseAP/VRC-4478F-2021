//the shitty drive functions
#include "drive.h"
#include "vex.h"

//stop all drive motors
void stopAllDrive(brakeType bT) {
  mLB.stop(bT);
  mLT.stop(bT);
  mRB.stop(bT);
  mRT.stop(bT); 
}
//spin the intakes
void spinIntakes(int s) {
  leftIntake.spin(forward, s, percent);
  rightIntake.spin(reverse, s, percent);
}
//stop the intakes
void stopIntakes(brakeType bt) {
  leftIntake.stop(bt);
  rightIntake.stop(bt);
}
//spin the rollers
void spinRollers(int s) {
  mainRoll.spin(forward, s, percent);
  finalRoll.spin(forward, s, percent);
}
//stop the rollers
void stopRollers(brakeType bt) {
  mainRoll.stop(bt);
  finalRoll.stop(bt);
}