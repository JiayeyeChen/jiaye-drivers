#ifndef HWT605_CAN_INCLINOMETER_H
#define HWT605_CAN_INCLINOMETER_H

#include "common.h"
#include <math.h>

#define					HWT605CAN_REG_SAVE 					  0x00
#define					HWT605CAN_REG_CALSW 					0x01
#define					HWT605CAN_REG_RSW 						0x02
#define					HWT605CAN_REG_RRATE					  0x03
#define					HWT605CAN_REG_BAUD 						0x04
#define					HWT605CAN_REG_AXOFFSET 			  0x05
#define					HWT605CAN_REG_AYOFFSET 				0x06
#define					HWT605CAN_REG_AZOFFSET 				0x07
#define					HWT605CAN_REG_GXOFFSET 				0x08
#define					HWT605CAN_REG_GYOFFSET 				0x09
#define					HWT605CAN_REG_GZOFFSET 				0x0A
#define					HWT605CAN_REG_HXOFFSET 				0x0B
#define					HWT605CAN_REG_HYOFFSET 				0x0C
#define					HWT605CAN_REG_HZOFFSET 				0x0D
#define					HWT605CAN_REG_IICADDR 				0x1A
#define					HWT605CAN_REG_LEDOFF					0x1B
#define					HWT605CAN_REG_MAGRANGX 				0x1C
#define					HWT605CAN_REG_MAGRANGY 				0x1D
#define					HWT605CAN_REG_MAGRANGZ 				0x1E
#define					HWT605CAN_REG_BANDWIDTH 			0x1F
#define					HWT605CAN_REG_GYRORANGE 			0x20
#define					HWT605CAN_REG_ACCRANGE 				0x21
#define					HWT605CAN_REG_SLEEP 					0x22
#define					HWT605CAN_REG_ORIENT 					0x23
#define					HWT605CAN_REG_AXIS6 					0x24
#define					HWT605CAN_REG_FILTK 					0x25
#define					HWT605CAN_REG_READADDR 				0x27
#define					HWT605CAN_REG_ACCFILT 				0x2A
#define					HWT605CAN_REG_POWONSEND 			0x2D
#define					HWT605CAN_REG_VERSION 				0x2E
#define					HWT605CAN_REG_YYMM 						0x30
#define					HWT605CAN_REG_DDHH 						0x31
#define					HWT605CAN_REG_MMSS 						0x32
#define					HWT605CAN_REG_MS 							0x33
#define					HWT605CAN_REG_AX 							0x34
#define					HWT605CAN_REG_AY 							0x35
#define					HWT605CAN_REG_AZ 							0x36
#define					HWT605CAN_REG_GX 							0x37
#define					HWT605CAN_REG_GY 							0x38
#define					HWT605CAN_REG_GZ 							0x39
#define					HWT605CAN_REG_HX 							0x3A
#define					HWT605CAN_REG_HY 							0x3B
#define					HWT605CAN_REG_HZ 							0x3C
#define					HWT605CAN_REG_ROLL 						0x3D
#define					HWT605CAN_REG_PITCH 					0x3E
#define					HWT605CAN_REG_YAW 						0x3F
#define					HWT605CAN_REG_TEMP 						0x40
#define					HWT605CAN_REG_PRESSUREL 			0x45
#define					HWT605CAN_REG_PRESSUREH 			0x46
#define					HWT605CAN_REG_HEIGHTL 				0x47
#define					HWT605CAN_REG_HEIGHTH 				0x48
#define					HWT605CAN_REG_Q0 							0x51
#define					HWT605CAN_REG_Q1 							0x52
#define					HWT605CAN_REG_Q2 							0x53
#define					HWT605CAN_REG_Q3 							0x54
#define					HWT605CAN_REG_GYROCALITHR 		0x61
#define					HWT605CAN_REG_ALARMLEVEL 			0x62
#define					HWT605CAN_REG_GYROCALTIME 		0x63
#define					HWT605CAN_REG_TRIGTIME 				0x68
#define					HWT605CAN_REG_KEY 						0x69
#define					HWT605CAN_REG_WERROR 					0x6A
#define					HWT605CAN_REG_WZTIME 					0x6E
#define					HWT605CAN_REG_WZSTATIC 				0x6F
#define					HWT605CAN_REG_XREFROLL 				0x79
#define					HWT605CAN_REG_YREFPITCH 			0x7A
#define					HWT605CAN_REG_NUMBERID1 			0x7F
#define					HWT605CAN_REG_NUMBERID2 			0x80
#define					HWT605CAN_REG_NUMBERID3 			0x81
#define					HWT605CAN_REG_NUMBERID4 			0x82
#define					HWT605CAN_REG_NUMBERID5 			0x83
#define					HWT605CAN_REG_NUMBERID6 			0x84

#define					HWT605CAN_DATATYPE_TIME				0x50
#define					HWT605CAN_DATATYPE_ACC 				0x51
#define					HWT605CAN_DATATYPE_GYRO				0x52
#define					HWT605CAN_DATATYPE_ANGLE 			0x53
#define					HWT605CAN_DATATYPE_MAG				0x54
#define					HWT605CAN_DATATYPE_PORT				0x55
#define					HWT605CAN_DATATYPE_PRESS			0x56
#define					HWT605CAN_DATATYPE_GPS				0x57
#define					HWT605CAN_DATATYPE_VELOCITY		0x58
#define					HWT605CAN_DATATYPE_QUATER			0x59
#define					HWT605CAN_DATATYPE_GSA				0x5A
#define					HWT605CAN_DATATYPE_READ 			0x5F

#define					HWT605_FEEDBACK_RATE_P2_HZ		0x01 //0.2Hz
#define					HWT605_FEEDBACK_RATE_P5_HZ		0x02
#define					HWT605_FEEDBACK_RATE_1_HZ			0x03
#define					HWT605_FEEDBACK_RATE_2_HZ			0x04
#define					HWT605_FEEDBACK_RATE_5_HZ			0x05
#define					HWT605_FEEDBACK_RATE_10_HZ		0x06
#define					HWT605_FEEDBACK_RATE_20_HZ		0x07
#define					HWT605_FEEDBACK_RATE_50_HZ    0x08
#define					HWT605_FEEDBACK_RATE_100_HZ   0x09
#define					HWT605_FEEDBACK_RATE_200_HZ   0x0B
#define					HWT605_FEEDBACK_RATE_ONCE			0x0C
#define					HWT605_FEEDBACK_RATE_NO				0x0D

#define					HWT605_CAN_BAUDRATE_1MHZ			0x00

typedef struct
{
	/*CAN bus*/
	CAN_HandleTypeDef*    hcan;
	uint8_t								canID;
	CAN_FilterTypeDef 		canFilter;
	uint8_t								rxbuf[8];
	uint8_t 							txbuf[8];
	uint32_t 							pTxMailbox;
  CAN_TxHeaderTypeDef   txHeader;
	float									feedbackCounter;
	/*Data*/
	union FloatUInt8			rawAccX;
	union FloatUInt8			rawAccY;
	union FloatUInt8			rawAccZ;
	union FloatUInt8			AccX;
	union FloatUInt8			AccY;
	union FloatUInt8			AccZ;
	union FloatUInt8			pitch;
	union FloatUInt8			yaw;
	float									rotCalibMatrixS11, rotCalibMatrixS12, rotCalibMatrixS13, \
												rotCalibMatrixS21, rotCalibMatrixS22, rotCalibMatrixS23, \
												rotCalibMatrixS31, rotCalibMatrixS32, rotCalibMatrixS33;
	float									biasCalibBX, biasCalibBY, biasCalibBZ;
	
	
}HWT605Handle;

HWT605Handle HWT605_Create(CAN_HandleTypeDef* hcan, uint8_t can_id, float calib_s11, float calib_s12, float calib_s13, \
													 float calib_s21, float calib_s22, float calib_s23, float calib_s31, float calib_s32, float calib_s33, \
														 float calib_bx, float calib_by, float calib_bz);

void HWT605_GetFeedback(HWT605Handle* himu, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[]);
void HWT605_ReadRegisterRequest(HWT605Handle* himu, uint8_t addr);
void HWT605_WriteRegister(HWT605Handle* himu, uint8_t addr, uint16_t data);
void HWT605_Save(HWT605Handle* himu);
void HWT605_Reboot(HWT605Handle* himu);
void HWT605_RecoverOriginalSettings(HWT605Handle* himu);
void HWT605_SetOutputContent(HWT605Handle* himu, uint8_t if_gsa, uint8_t if_quater, \
														 uint8_t if_vel, uint8_t if_gps, uint8_t if_press, \
														 uint8_t if_port, uint8_t if_mag, uint8_t if_angle, \
														 uint8_t if_gyro, uint8_t if_acc, uint8_t if_time);
void HWT605_SetFeedbackRate(HWT605Handle* himu, uint16_t rate_data);
void HWT605_SetCANBaudrate(HWT605Handle* himu, uint16_t baudrate_data);
extern HWT605Handle himu;


#endif
