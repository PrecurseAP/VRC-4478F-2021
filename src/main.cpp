#include "vex.h"
#include "util.h"
#include "pathing.h"
#include "odometry.h"
#include "ppc.h"

using namespace vex;

competition Competition;

pathing::Point p1 = pathing::Point( 0 , 0 );
pathing::Point p2 = pathing::Point( 5 , 0 );
pathing::Point p3 = pathing::Point( 5 , 5 );
pathing::Point p4 = pathing::Point( 0 , 5 );

pathing::Path mainPath(p1, p2, p3, p4, 20); 

void pre_auton(void) {
  vexcodeInit();
}

void autonomous(void) {

}

void usercontrol(void) {
  while (1) {
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
