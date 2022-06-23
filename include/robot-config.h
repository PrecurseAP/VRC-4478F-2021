using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor mFrontLeft;
extern motor mMidLeft;
extern motor mBackLeft;
extern motor mFrontRight;
extern motor mMidRight;
extern motor mBackRight;
extern motor mLift;
extern motor mConveyor;
extern controller Controller2;
extern digital_out tilterPiston;
extern digital_out clawPiston;
extern inertial GYRO;
extern rotation twLeft;
extern rotation twRight;
extern rotation twBack;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );