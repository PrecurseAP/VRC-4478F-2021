#ifndef __ODOMETRY
#define __ODOMETRY

extern void turnMoveToPoint(float, float, int);
extern void spotTurn(float, int);
extern void moveForward(float, int);
extern void move(float, int);
extern void lowerTilter(bool);
extern void raiseTilter(bool);
extern void raiseLift(bool);
extern void lowerLift(bool);
extern void raiseTilterWithGoal(bool);
extern void spinConveyor(void);
#endif