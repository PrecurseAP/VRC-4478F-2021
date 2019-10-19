using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftFront;
extern motor rightFront;
extern motor leftArm;
extern motor rightArm;
extern motor clawMotor;
extern motor clawExtendLeft;
extern motor clawExtendRight;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );