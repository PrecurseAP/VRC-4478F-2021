#ifndef __MOVEMENTS__
#define __MOVEMENTS__

#include "vex.h"

extern void basicDrive(int);

extern bool clawState;

extern void clawToggle(void);

extern void raiseLift(int val = 60, bool wait = false);

extern int deploy();

extern void lowerLift();

extern void raiseLiftFully(bool wait = true);

extern void spinConveyor(void);

extern void lowerTilter(int speed = 100, int val = -540, bool wait = true);

extern void raiseTilterWithGoal(bool wait = true);

extern void moveRightSide(int);

extern void moveLeftSide(int);

extern void stopAllDrive(brakeType);

extern int turnToAngle(float, float, float kp = .715, float ki = 0.000, float kd = .19);

extern int turnWithTilterGoal(float, float, float kp = .65, float ki = 0.000014, float kd = .49);

extern int turnWith2Goals(float, float, float kp = .55, float ki = 0.000011, float kd = .55);

extern int turnWithClawGoal(float, float, float kp = .595, float ki = 0.00001, float kd = .35);

extern int moveStraight(float, float, float kp = 6.0, float ki = 0.002, float kd = 3.6);

extern void raiseTilter(bool wait = true);

extern int depositAndDrop();

extern float driveRatio;

#endif