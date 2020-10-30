#ifndef __AIDENMATH__
#define __AIDENMATH__
//dont question why i made this
#include <climits>
#include <math.h>
//sign values 

template <class T>
int sign(T val) {
  return val > 0 ? 1 : -1;
}

template <class T>
T MAX(T INPUT, T INPUT2, T INPUT3 = INT_MIN, T INPUT4 = INT_MIN) {
  T _temp = INPUT > INPUT2 ? INPUT : INPUT2; 
  T _temp2 = INPUT3 > INPUT4 ? INPUT3 : INPUT4;
  return _temp > _temp2 ? _temp : _temp2;
}

template <class T>
float distanceFormula(T x1, T y1, T x2, T y2) {
  return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2));
}
#endif //__AIDENMATH__
