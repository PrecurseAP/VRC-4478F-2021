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
motor mRUpper = motor(PORT20, ratio18_1, true);
motor mRLower = motor(PORT19, ratio18_1, false);
gps GPS = gps(PORT5, 0.00, 0.00, mm, 180);
motor mConveyor = motor(PORT18, ratio18_1, true);
motor mTray = motor(PORT1, ratio36_1, false);
motor mArm = motor(PORT13, ratio18_1, false);
digital_out clawPiston = digital_out(Brain.ThreeWirePort.A);
inertial GYRO = inertial(PORT7);

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