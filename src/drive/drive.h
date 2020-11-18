#ifndef __DRIVE__
#define __DRIVE__

#include "vex.h"

extern void _drive(void);
extern void resetGyro(void);
extern void stopAllDrive(brakeType);
extern void spinMotors(int, int, int, int);
extern void driveNew(void);
extern float logDrive_shallow(float);
extern float logDrive_shallowish(float);
extern float logDrive_steepish(float);
extern float logDrive_steep(float);

extern float goalAngle, angleIntegral, angleError;
extern float normalizer, angleDerivative, previousError;
extern float kP, kI, kD;                      
extern float mSpd[4];
extern float turnValue;
extern bool autoTurn;
extern double maxAxis, maxOutput;
extern float gyroAngle;
extern int joyX, joyY;
extern float joyZ;
extern float vel;
extern float x2;
extern float y2;
extern float datanx2y2;
extern float sqrtx2y2;

#endif //__DRIVE__