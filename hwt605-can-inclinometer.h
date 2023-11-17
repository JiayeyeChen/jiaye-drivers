#ifndef HWT605_CAN_INCLINOMETER_H
#define HWT605_CAN_INCLINOMETER_H

#include "common.h"

typedef struct
{
	CAN_HandleTypeDef*    hcan;
	uint8_t								canID;
	
	
}HWT605Handle;

HWT605Handle HWT605_Create(CAN_HandleTypeDef* hcan, uint8_t can_id);







extern HWT605Handle himu;


#endif
