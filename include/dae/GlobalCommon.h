#ifndef __GLOBAL_COMMON_H_INCLUDED__
#define __GLOBAL_COMMON_H_INCLUDED__

#define _USE_MATH_DEFINES
#include <math.h>


//#define HALF_ANGLE                      15          //half angle in degree

extern double gHalfangle;
extern double gSinHalfangle;
extern double gCosHalfangle;
extern double gSinDoubleHalfangle;
extern double gCosDoubleHalfangle;

extern double gSinMinusHalfangle;
extern double gCosMinusHalfangle;
extern double gSinMinusDoubleHalfangle;
extern double gCosMinusDoubleHalfangle;

void initializeGlobal();



#endif
