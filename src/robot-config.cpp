#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor mLT = motor(PORT10, ratio18_1, false);
motor mLB = motor(PORT2, ratio18_1, false);
motor mRT = motor(PORT3, ratio18_1, false);
motor mRB = motor(PORT4, ratio18_1, false);
controller Controller1 = controller(primary);
motor leftIntake = motor(PORT20, ratio6_1, false);
motor rightIntake = motor(PORT5, ratio6_1, false);
motor mainRoll = motor(PORT6, ratio6_1, false);
motor finalRoll = motor(PORT11, ratio6_1, true);
rotation rightEncoder = rotation(PORT7, true);
rotation leftEncoder = rotation(PORT9, false);
inertial GYRO = inertial(PORT8);

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