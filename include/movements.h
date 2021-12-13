#ifndef __MOVEMENTS__
#define __MOVEMENTS__

#include "vex.h"

extern bool clawState;

extern void clawToggle(void);

extern void raiseLift(int val = 100, bool wait = false);

extern int deploy(int val = 100);

extern void lowerLift(bool wait = false);

extern void spinConveyor(void);

extern void lowerTilter(int speed = 100, int val = -540, bool wait = false);

extern void moveRightSide(int);

extern void moveLeftSide(int);

extern void stopAllDrive(brakeType);

extern int turnToAngle(float, int, float kp = .64, float ki = 0.0, float kd = .19);

extern int moveStraight(float, int, float kp = .64, float ki = 0.0, float kd = .19);

#endif