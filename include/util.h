#ifndef __UTIL
#define __UTIL

#include "math.h"

template <class T> 
T sq(T i) {
  //square function, nice and small
  return (i*i);
}

template <class U>
U cb(U i) {
  //cube function, also nice and small
  return (i*i*i);
}

#endif