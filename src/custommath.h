#ifndef __AIDENMATH__
#define __AIDENMATH__
//dont question why i made this
#include <climits>
//sign values 

template <class S>
S sign(S val) {
  return val > 0 ? 1 : -1;
}

template <class T>
T MAX(T INPUT, T INPUT2, T INPUT3 = INT_MIN, T INPUT4 = INT_MIN) {
  T _temp = INPUT > INPUT2 ? INPUT : INPUT2; 
  T _temp2 = INPUT3 > INPUT4 ? INPUT3 : INPUT4;
  return _temp > _temp2 ? _temp : _temp2;
}

template <class U>
U pointDistance(U x1, U y1, U x2, U y2) {
  
}
#endif //__AIDENMATH__
