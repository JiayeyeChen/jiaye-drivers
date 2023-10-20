#include "xiaomi_cybergear.h"


struct exCanIdInfo{ 
uint32_t id:8; 
uint32_t data:16; 
uint32_t mode:5; 
uint32_t res:3; 
};

struct exCanIdInfo struct1111;


CybergearHandle CYBERGEAR_Create(CAN_HandleTypeDef* hcan, uint8_t motor_can_id, uint8_t master_can_id, float direction)
{
	CybergearHandle hmotor;
	hmotor.hcan = hcan;
	hmotor.masterCANID = master_can_id;
	hmotor.motorCANID = motor_can_id;
	hmotor.task = CYBERGEAR_TASK_SHUTDOWN;
	hmotor.realCur.f = 0.0f;
	hmotor.realPosRad.f = 0.0f;
	hmotor.realPosDeg.f = 0.0f;
	hmotor.realVelRad.f = 0.0f;
	hmotor.realVelDeg.f = 0.0f;
	hmotor.realTorque.f = 0.0f;
	hmotor.temperature.f = 0.0f;
	hmotor.ifError = 0;
	hmotor.hallEncoderError = 0;
	hmotor.ifOverheat = 0;
	hmotor.ifOvercurrent = 0;
	hmotor.ifUndervoltage = 0;
	hmotor.mode = 0;
	hmotor.directionCorrection = direction;
	memset(hmotor.txBuf, 0, 8);
	
	return hmotor;
}

void CYBERGEAR_Send(CybergearHandle* hmotor, uint8_t cmd, uint16_t data_extended, uint8_t dataLen, uint8_t data[])
{
	hmotor->txHeader.DLC = dataLen;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.RTR = 0;
	hmotor->txHeader.ExtId = hmotor->motorCANID | data_extended<<8 | cmd << 24;
	if (dataLen)
		memcpy(hmotor->txBuf, data, dataLen);
	HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, &hmotor->pTxMailbox);
}


void CYBERGEAR_Enable(CybergearHandle* hmotor)
{
	uint8_t data_dump[8] = {0,0,0,0,0,0,0,0};
	CYBERGEAR_Send(hmotor, CYBERGEAR_COMMAND_ENABLE, hmotor->masterCANID, 0, data_dump);
}

void CYBERGEAR_Disable(CybergearHandle* hmotor)
{
	uint8_t data_dump[8] = {0,0,0,0,0,0,0,0};
	CYBERGEAR_Send(hmotor, CYBERGEAR_COMMAND_DISABLE, hmotor->masterCANID, 0, data_dump);
}

void CYBERGEAR_GeneralControl(CybergearHandle* hmotor, float torque, float posRad, float velRad, float kp, float kd)
{
	LIMIT_MIN_MAX(torque, -12.0f, 12.0f);
	LIMIT_MIN_MAX(posRad, -4.0f*pi, 4.0f*pi);
	LIMIT_MIN_MAX(velRad, -30.0f, 30.0f);
	LIMIT_MIN_MAX(kp, 0.0f, 500.0f);
	LIMIT_MIN_MAX(kd, 0.0f, 5.0f);
	uint16_t torqueInt = (uint16_t)(torque * 2730.625f + 32767.5f);
	uint16_t posInt = (uint16_t)(posRad * (65535.0f/(8.0f*pi)) + 32767.5f);
	uint16_t velInt = (uint16_t)(velRad * (65535.0f/60.0f) + 32767.5f);
	uint16_t kpInt = (uint16_t)(kp * (65535.0f/500.0f));
	uint16_t kdInt = (uint16_t)(kd * (65535.0f/5.0f));
	uint8_t data[8];
	data[0] = (posInt >> 8) & 0xFF;
	data[1] = posInt & 0xFF;
	data[2] = (velInt >> 8) & 0xFF;
	data[3] = velInt & 0xFF;
	data[4] = (kpInt >> 8) & 0xFF;
	data[5] = kpInt & 0xFF;
	data[6] = (kdInt >> 8) & 0xFF;
	data[7] = kdInt & 0xFF;
	
	CYBERGEAR_Send(hmotor, CYBERGEAR_COMMAND_GENERAL_CONTROL, torqueInt, 8, data);
}

void CYBERGEAR_GetFeedback(CybergearHandle* hmotor, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[])
{
	if (((rxheader->ExtId >> 24) & 0x0F) == 2) //Feedback type 2
	{
		hmotor->ifError = (rxheader->ExtId >> 16) & 0x3F;
		hmotor->hallEncoderError = (rxheader->ExtId >> 20) & 0x01;
		hmotor->ifOverheat = (rxheader->ExtId >> 18) & 0x01;
		hmotor->ifOvercurrent = (rxheader->ExtId >> 17) & 0x01;
		hmotor->ifUndervoltage = (rxheader->ExtId >> 16) & 0x01;
		hmotor->mode = (rxheader->ExtId >> 22) & 0x03;
		hmotor->realPosRad.f = (((float)(rxbuf[1] | (rxbuf[0] << 8))) * 8.0f * pi/65535.0f - 4.0f * pi )* hmotor->directionCorrection;
		hmotor->realPosDeg.f = hmotor->realPosRad.f * rad2deg;
		hmotor->realVelRad.f = (((float)(rxbuf[3] | (rxbuf[2] << 8))) * 60.0f/65535.0f - 30.0f) * hmotor->directionCorrection;
		hmotor->realVelDeg.f = hmotor->realVelRad.f * rad2deg;
		hmotor->realTorque.f = ((float)(rxbuf[5] | (rxbuf[4] << 8))) * 24.0f/65535.0f - 12.0f;
		hmotor->temperature.f = ((float)(rxbuf[7] | (rxbuf[6] << 8))) / 10.0f;
	}
}

void CYBERGEAR_Zeroing(CybergearHandle* hmotor)
{
	uint8_t data[8] = {1,0,0,0,0,0,0,0};
	CYBERGEAR_Send(hmotor, CYBERGEAR_COMMAND_ZEROING, hmotor->masterCANID, 8, data);
}


uint8_t CYBERGEAR_CheckCANRxHeader(CybergearHandle* hmotor, CAN_RxHeaderTypeDef rxheader)
{
	if ((rxheader.IDE == CAN_ID_EXT) && \
		  ((rxheader.ExtId & 0xFF) == hmotor->masterCANID) && \
			((rxheader.ExtId >> 8) & 0xFF) == hmotor->motorCANID)
		return 1;
	else
		return 0;
}

void CYBERGEAR_SetCANIDs(CybergearHandle* hmotor, uint8_t motor_id, uint8_t master_id)
{
	uint8_t data_dump[8] = {0,0,0,0,0,0,0,0};
	uint16_t data_extended = master_id | (motor_id << 8);
	CYBERGEAR_Send(hmotor, CYBERGEAR_COMMAND_SET_CAN_ID, data_extended, 8, data_dump);
}
