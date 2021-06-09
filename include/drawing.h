#ifndef __DRAWING
#define __DRAWING

#include "vex.h"

namespace drawing {
  enum autonSquare {
    leftRed,
    rightRed,
    leftBlue,               //enum to simplify human readability of autonomous start positions
    rightBlue,
    none
  };

  struct preautonStats {
    int tx;
    int ty;                             //structure to act as a container for touch screen variables
    autonSquare touchedSquare = none;
    bool autSel = false;
  };

  void renderScreen(void);
  void drawGoal(int, int, color);
  void touchScreenLogic(void);
}

#endif