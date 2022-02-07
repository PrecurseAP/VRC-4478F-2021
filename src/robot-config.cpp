#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor mFrontLeft = motor(PORT1, ratio6_1, true);
motor mMidLeft = motor(PORT2, ratio6_1, true);
motor mBackLeft = motor(PORT3, ratio6_1, true);
motor mFrontRight = motor(PORT11, ratio6_1, false);
motor mMidRight = motor(PORT12, ratio6_1, false);
motor mBackRight = motor(PORT13, ratio6_1, false);
motor mLift = motor(PORT18, ratio36_1, false);
motor mConveyor = motor(PORT19, ratio6_1, true);
controller Controller2 = controller(partner);
digital_out tilterPiston = digital_out(Brain.ThreeWirePort.A);
digital_out clawPiston = digital_out(Brain.ThreeWirePort.H);
inertial GYRO = inertial(PORT10);

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