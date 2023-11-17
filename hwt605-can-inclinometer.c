#include "hwt605-can-inclinometer.h"

HWT605Handle himu;

HWT605Handle HWT605_Create(CAN_HandleTypeDef* hcan, uint8_t can_id)
{
	HWT605Handle himu;
	
	himu.hcan = hcan;
	himu.canID = can_id;
	
	return himu;
}
