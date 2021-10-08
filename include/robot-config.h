using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor mLUpper;
extern motor mLLower;
extern motor mRUpper;
extern motor mRLower;
extern gps GPS;
extern motor mConveyor;
extern motor mTray;
extern motor mArm;
extern digital_out clawPiston;
extern inertial GYRO;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );