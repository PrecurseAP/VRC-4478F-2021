#include "vex.h"
#include "custommath.h"

extern void stopAllDrive(brakeType);
extern void spinIntakes(int);
extern void stopIntakes(brakeType);
extern void spinRollers(int);
extern void stopRollers(brakeType);

extern void moveStraight(float, float = 100.0, float = 1.0);
extern void turnToAngle(float, float = 3.0);

extern const float wheelC;
