#ifndef __ODOMETRY
#define __ODOMETRY

extern void turnMoveToPoint(float, float, int);
extern void spotTurn(float, int, int, int);
extern void moveForward(float, int);
extern void move(float, int, int, int);
extern void lowerTilter(bool);
extern void raiseTilter(bool);
extern void raiseLift(bool);
extern void lowerLift(bool);
extern void raiseTilterWithGoal(bool);
extern void spinConveyor(void);
extern void lowerTilterWithValue(bool, int);
extern void spotTurnWithTilterGoal(float, int, int, int);
extern void spotTurnWithClawGoal(float, int, int, int);
extern void spotTurnWith2Goals(float, int, int, int);
extern void spotTurnWithNoAngleWrap(float, int, int, int);
extern void moveSlow(float, int, int, int);
extern void lowerTilterSlow(bool);
#endif