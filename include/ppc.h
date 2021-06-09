#ifndef __PPC
#define __PPC

#include "math.h"
#include "util.h"
#include "pathing.h"
#include "vex.h"

namespace ppc {
  extern void loop(pathing::Path*);
  extern float lookahead;
  extern float lookaheadKF;
  extern float calcAngleError(float, float);
}

#endif