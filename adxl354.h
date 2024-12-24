#ifndef ADXL354_H
#define ADXL354_H

#include "common.h"

typedef struct
{
	LowPassFilterHandle hfilterX, hfilterY, hfilterZ;
	float rawVoltX, rawVoltY, rawVoltZ, raw1P8ANA, rawTemp;
	float rawX,rawY,rawZ, rawMag;
	float filteredX, filteredY, filteredZ, filteredMag;
	float g;
}ADXL354Handle;

ADXL354Handle ADXL354_Create(float lp_filter_cut_off_frequency, float lp_filter_period_in_second);
void ADXL354_UpdateVoltMeas(ADXL354Handle* hadxl, float x, float y, float z, float ref1p8ana, float temp);
void ADXL354_UpdateAcceleration(ADXL354Handle* hadxl);

#endif
