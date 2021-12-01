#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor mLUpper = motor(PORT12, ratio18_1, false);
motor mLLower = motor(PORT11, ratio18_1, true);
motor mRUpper = motor(PORT15, ratio18_1, true);
motor mRLower = motor(PORT19, ratio18_1, false);
gps GPS = gps(PORT16, 120.65, 63.50, mm, 90);
motor mConveyor = motor(PORT18, ratio18_1, true);
motor mLTray = motor(PORT1, ratio36_1, true);
motor mArm = motor(PORT13, ratio36_1, false);
digital_out clawPiston = digital_out(Brain.ThreeWirePort.F);
inertial GYRO = inertial(PORT17);
motor mRTray = motor(PORT14, ratio36_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}