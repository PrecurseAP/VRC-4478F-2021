#ifndef __MOVEMENTS__
#define __MOVEMENTS__

#include "vex.h"

extern bool clawState;

extern void clawToggle(void);

extern void raiseLift(int val = 100, bool wait = false);

extern int deploy();

extern void lowerLift();

extern void raiseLiftFully(bool wait = true);

extern void spinConveyor(void);

extern void lowerTilter(int speed = 100, int val = -540, bool wait = true);

extern void raiseTilterWithGoal(bool wait = true);

extern void moveRightSide(int);

extern void moveLeftSide(int);

extern void stopAllDrive(brakeType);

extern int turnToAngle(float, int, float kp = .67, float ki = 0.0, float kd = .19);

extern int turnWithTilterGoal(float, int, float kp = .58, float ki = 0.00001, float kd = .29);

extern int turnWith2Goals(float, int, float kp = .50, float ki = 0.000011, float kd = .55);

extern int turnWithClawGoal(float, int, float kp = .55, float ki = 0.00001, float kd = .35);

extern int moveStraight(float, int, float kp = 7.0, float ki = 0.000035, float kd = 1.9);

extern void raiseTilter(bool wait = true);

extern int depositAndDrop();

#endif