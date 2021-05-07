using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor mLT;
extern motor mLB;
extern motor mRT;
extern motor mRB;
extern controller Controller1;
extern motor leftIntake;
extern motor rightIntake;
extern motor mainRoll;
extern motor finalRoll;
extern rotation rightEncoder;
extern rotation leftEncoder;
extern inertial GYRO;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );