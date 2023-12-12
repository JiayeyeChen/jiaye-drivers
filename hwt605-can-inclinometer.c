#include "hwt605-can-inclinometer.h"

HWT605Handle himu;

HWT605Handle HWT605_Create(CAN_HandleTypeDef* hcan, uint8_t can_id, float calib_s11, float calib_s12, float calib_s13, \
													 float calib_s21, float calib_s22, float calib_s23, float calib_s31, float calib_s32, float calib_s33, \
														 float calib_bx, float calib_by, float calib_bz)
{
	HWT605Handle himu;
	
	himu.hcan = hcan;
	himu.canID = can_id;
	himu.feedbackCounter = 0.0f;
	himu.rotCalibMatrixS11 = calib_s11;
	himu.rotCalibMatrixS12 = calib_s12;
	himu.rotCalibMatrixS13 = calib_s13;
	himu.rotCalibMatrixS21 = calib_s21;
	himu.rotCalibMatrixS22 = calib_s22;
	himu.rotCalibMatrixS23 = calib_s23;
	himu.rotCalibMatrixS31 = calib_s31;
	himu.rotCalibMatrixS32 = calib_s32;
	himu.rotCalibMatrixS33 = calib_s33;
	himu.biasCalibBX = calib_bx;
	himu.biasCalibBY = calib_by;
	himu.biasCalibBZ = calib_bz;
	return himu;
}

void HWT605_GetFeedback(HWT605Handle* himu, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[])
{
	memcpy(himu->rxbuf, rxbuf, 8);
	if (rxbuf[1] == HWT605CAN_DATATYPE_TIME)
	{
	}
	else if (rxbuf[1] == HWT605CAN_DATATYPE_ACC)
	{
		himu->rawAccX.f = ((float)((int16_t)(rxbuf[2] | rxbuf[3] << 8))) / 32768.0f * 16.0f * 9.8f;
		himu->rawAccY.f = ((float)((int16_t)(rxbuf[4] | rxbuf[5] << 8))) / 32768.0f * 16.0f * 9.8f;
		himu->rawAccZ.f = ((float)((int16_t)(rxbuf[6] | rxbuf[7] << 8))) / 32768.0f * 16.0f * 9.8f;
		himu->AccX.f = himu->rotCalibMatrixS11 * (himu->rawAccX.f - himu->biasCalibBX) + \
									 himu->rotCalibMatrixS12 * (himu->rawAccY.f - himu->biasCalibBY) + \
									 himu->rotCalibMatrixS13 * (himu->rawAccZ.f - himu->biasCalibBZ);
		
		himu->AccY.f = himu->rotCalibMatrixS21 * (himu->rawAccX.f - himu->biasCalibBX) + \
									 himu->rotCalibMatrixS22 * (himu->rawAccY.f - himu->biasCalibBY) + \
									 himu->rotCalibMatrixS23 * (himu->rawAccZ.f - himu->biasCalibBZ);
		
		himu->AccZ.f = himu->rotCalibMatrixS31 * (himu->rawAccX.f - himu->biasCalibBX) + \
									 himu->rotCalibMatrixS32 * (himu->rawAccY.f - himu->biasCalibBY) + \
									 himu->rotCalibMatrixS33 * (himu->rawAccZ.f - himu->biasCalibBZ);
		himu->feedbackCounter += 0.005f;
	}
	else if (rxbuf[1] == HWT605CAN_DATATYPE_GYRO)
	{
	}
	else if (rxbuf[1] == HWT605CAN_DATATYPE_ANGLE)
	{
	}
	else if (rxbuf[1] == HWT605CAN_DATATYPE_MAG)
	{
	}
	else if (rxbuf[1] == HWT605CAN_DATATYPE_READ)
	{
	}
	
}

void HWT605_ReadRegisterRequest(HWT605Handle* himu, uint8_t addr)
{
	himu->txHeader.DLC = 5;
  himu->txHeader.IDE = CAN_ID_STD;
  himu->txHeader.RTR = 0;
	himu->txHeader.StdId = himu->canID;
	himu->txbuf[0] = 0xFF;
	himu->txbuf[1] = 0xAA;
	himu->txbuf[2] = HWT605CAN_REG_READADDR;
	himu->txbuf[3] = addr;
	himu->txbuf[4] = 0;
	HAL_CAN_AddTxMessage(himu->hcan, &(himu->txHeader), himu->txbuf, &himu->pTxMailbox);
}

void HWT605_WriteRegister(HWT605Handle* himu, uint8_t addr, uint16_t data)
{
	himu->txHeader.DLC = 5;
  himu->txHeader.IDE = CAN_ID_STD;
  himu->txHeader.RTR = 0;
	himu->txHeader.StdId = himu->canID;
	himu->txbuf[0] = 0xFF;
	himu->txbuf[1] = 0xAA;
	himu->txbuf[2] = addr;
	himu->txbuf[3] = (uint8_t)(data & 0xFF);
	himu->txbuf[4] = (uint8_t)((data >> 8) & 0xFF);
	HAL_CAN_AddTxMessage(himu->hcan, &(himu->txHeader), himu->txbuf, &himu->pTxMailbox);
}

void HWT605_Save(HWT605Handle* himu)
{
	HWT605_WriteRegister(himu, HWT605CAN_REG_SAVE, 0x0000);
}
void HWT605_Reboot(HWT605Handle* himu)
{
	HWT605_WriteRegister(himu, HWT605CAN_REG_SAVE, 0x00FF);
}
void HWT605_RecoverOriginalSettings(HWT605Handle* himu)
{
	HWT605_WriteRegister(himu, HWT605CAN_REG_SAVE, 0x0001);
}

void HWT605_SetOutputContent(HWT605Handle* himu, uint8_t if_gsa, uint8_t if_quater, \
													uint8_t if_vel, uint8_t if_gps, uint8_t if_press, \
													uint8_t if_port, uint8_t if_mag, uint8_t if_angle, \
													uint8_t if_gyro, uint8_t if_acc, uint8_t if_time)
{
	uint16_t data = if_time | if_acc << 1 | if_gyro << 2 | if_angle << 3 | \
													 if_mag << 4 | if_port << 5 | if_press << 6 | \
													 if_gps << 7 | if_vel  << 8 | if_quater << 9| \
													 if_gsa << 10;
	HWT605_WriteRegister(himu, HWT605CAN_REG_RSW, data);
}

void HWT605_SetFeedbackRate(HWT605Handle* himu, uint16_t rate_data)
{
	HWT605_WriteRegister(himu, HWT605CAN_REG_RRATE, rate_data);
}

void HWT605_SetCANBaudrate(HWT605Handle* himu, uint16_t baudrate_data)
{
	HWT605_WriteRegister(himu, HWT605CAN_REG_BAUD, baudrate_data);
}
