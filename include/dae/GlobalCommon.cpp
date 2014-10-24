#include "GlobalCommon.h"

double gHalfangle;
double gSinHalfangle;
double gCosHalfangle;
double gSinDoubleHalfangle;
double gCosDoubleHalfangle;
double gSinMinusHalfangle;
double gCosMinusHalfangle;
double gSinMinusDoubleHalfangle;
double gCosMinusDoubleHalfangle;

void initializeGlobal() {
    //gHalfangle = HALF_ANGLE * M_PI / 180.0f;
    gHalfangle = 10 * M_PI / 180.0f;
    gSinHalfangle = sin(gHalfangle * 1);
    gCosHalfangle = cos(gHalfangle * 1);
    gSinDoubleHalfangle = sin(gHalfangle * 2);
    gCosDoubleHalfangle = cos(gHalfangle * 2);
    gSinMinusHalfangle = sin(gHalfangle * -1);
    gCosMinusHalfangle = cos(gHalfangle * -1);
    gSinMinusDoubleHalfangle = sin(gHalfangle * -2);
    gCosMinusDoubleHalfangle = cos(gHalfangle * -2);
}
