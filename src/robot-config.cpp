#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller = controller(primary);
motor mLFront = motor(PORT1, ratio18_1, false);
motor mRFront = motor(PORT2, ratio18_1, false);
motor mLBack = motor(PORT3, ratio18_1, false);
motor mRBack = motor(PORT4, ratio18_1, false);
rotation twLeft = rotation(PORT5, false);
rotation twRight = rotation(PORT6, false);


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