#ifndef RPM_CALC_H
#define RPM_CALC_H

#include <stdint.h>

void startADC();

float getRPS();
float calculateCurrentPositionInRadians();

#endif