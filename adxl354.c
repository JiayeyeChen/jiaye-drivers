#include "adxl354.h"


ADXL354Handle ADXL354_Create(float lp_filter_cut_off_frequency, float lp_filter_period_in_second)
{
	ADXL354Handle hadxl354;
	hadxl354.raw1P8ANA = 0.0f;
	hadxl354.rawTemp = 0.0f;
	hadxl354.rawVoltX = 0.0f;
	hadxl354.rawVoltY = 0.0f;
	hadxl354.rawVoltZ = 0.0f;
	hadxl354.rawX = 0.0f;
	hadxl354.rawY = 0.0f;
	hadxl354.rawZ = 0.0f;
	hadxl354.rawMag = 0.0f;
	hadxl354.filteredX = 0.0f;
	hadxl354.filteredY = 0.0f;
	hadxl354.filteredZ = 0.0f;
	hadxl354.g = 9.781f; // Singapore gravity acceleration
	LowPassFilter_Init(&hadxl354.hfilterX, lp_filter_cut_off_frequency, lp_filter_period_in_second);
	LowPassFilter_Init(&hadxl354.hfilterY, lp_filter_cut_off_frequency, lp_filter_period_in_second);
	LowPassFilter_Init(&hadxl354.hfilterZ, lp_filter_cut_off_frequency, lp_filter_period_in_second);
	return hadxl354;
}

void ADXL354_UpdateVoltMeas(ADXL354Handle* hadxl, float x, float y, float z, float ref1p8ana, float temp)
{
	hadxl->rawVoltX = x;
	hadxl->rawVoltY = y;
	hadxl->rawVoltZ = z;
	hadxl->raw1P8ANA = ref1p8ana;
	hadxl->rawTemp = temp;
}

void ADXL354_UpdateAcceleration(ADXL354Handle* hadxl)
{
	float acc_per_volt = 4.0f * hadxl->g / hadxl->raw1P8ANA;
	hadxl->rawX = 2.0f * hadxl->g - acc_per_volt * hadxl->rawVoltX;
	hadxl->rawY = 2.0f * hadxl->g - acc_per_volt * hadxl->rawVoltY;
	hadxl->rawZ = 2.0f * hadxl->g - acc_per_volt * hadxl->rawVoltZ;
	hadxl->rawMag = hadxl->rawX*hadxl->rawX + hadxl->rawY * hadxl->rawY + hadxl->rawZ * hadxl->rawZ;
	hadxl->rawMag = sqrtf(hadxl->rawMag);
	LowPassFilter_Update(&hadxl->hfilterX, hadxl->rawX);
	LowPassFilter_Update(&hadxl->hfilterY, hadxl->rawY);
	LowPassFilter_Update(&hadxl->hfilterZ, hadxl->rawZ);
	hadxl->filteredX = hadxl->hfilterX.output.f;
	hadxl->filteredY = hadxl->hfilterY.output.f;
	hadxl->filteredZ = hadxl->hfilterZ.output.f;
	hadxl->filteredMag = hadxl->filteredX*hadxl->filteredX+hadxl->filteredY*hadxl->filteredY+hadxl->filteredZ*hadxl->filteredZ;
	hadxl->filteredMag = sqrtf(hadxl->filteredMag);
}
