using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller;
extern motor mLFront;
extern motor mRFront;
extern motor mLBack;
extern motor mRBack;
extern rotation twLeft;
extern rotation twRight;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );