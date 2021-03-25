#include "drive.h"
#include "vex.h"

void stopAllDrive(brakeType bT) {
  mLB.stop(bT);
  mLT.stop(bT);
  mRB.stop(bT);
  mRT.stop(bT); 
}