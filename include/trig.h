#ifndef __HAHAHA__
#define __HAHAHA__ //@FuzzySn0wman

#include <math.h>

double radToDeg = 57.2957795130823;
double degToRad = 0.01745329251994;

/* dsin function returns sine of INPUT but input and output are in degrees, same goes for all other functions */
template <class T>
float dsin(T INPUT) { return (sin(INPUT*degToRad)); }

/* dcos function */
template <class T>
float dcos(T INPUT) { return (cos(INPUT*degToRad)); }

/* dtan function */
template <class T> 
float dtan(T INPUT) { return (tan(INPUT*degToRad)); }

/* atan2 function */
template <class T>
float datan2(T INPUT, T INPUT2) { return (atan2((float)INPUT, (float)INPUT2)*radToDeg); }

/* dacos function */
template <class T> 
float dacos(T INPUT) { return acos(INPUT)*radToDeg; }

/* dasin function */
template <class T>
float dasin(T INPUT) { return asin(INPUT)*radToDeg; }

/* atan functions */
template <class T>
float datan(T INPUT) { return atan(INPUT)*radToDeg; }

#define toRadians(INPUT) \
  ((INPUT) * degToRad)

#define toDegrees(INPUT) \
  ((INPUT) * radToDeg)

#endif //__HAHAHA__