#ifndef XIAOMI_CYBERGEAR_H
#define XIAOMI_CYBERGEAR_H

#include "common.h"



#define P_MIN -12.5f 
#define P_MAX 12.5f 
#define V_MIN -30.0f 
#define V_MAX 30.0f 
#define KP_MIN 0.0f 
#define KP_MAX 500.0f 
#define KD_MIN 0.0f 
#define KD_MAX 5.0f 
#define T_MIN -12.0f 
#define T_MAX 12.0f 


#define CYBERGEAR_COMMAND_GENERAL_CONTROL    0x01
#define CYBERGEAR_COMMAND_ENABLE						 0x03
#define CYBERGEAR_COMMAND_DISABLE						 0x04
#define CYBERGEAR_COMMAND_ZEROING						 0x06
#define CYBERGEAR_COMMAND_SET_CAN_ID				 0x07




enum CYBERGEAR_TASK
{
	CYBERGEAR_TASK_IDLE,
	CYBERGEAR_TASK_SHUTDOWN,
	CYBERGEAR_TASK_POSITION_CONTROL,
	CYBERGEAR_TASK_VELOCITY_CONTROL,
	CYBERGEAR_TASK_CURRENT_CONTROL
};

typedef struct
{
	/*CAN bus data*/
	CAN_HandleTypeDef*    hcan;
	uint8_t								masterCANID;
	uint8_t								motorCANID;
	uint8_t								txBuf[8];
	uint32_t              pTxMailbox;
  CAN_TxHeaderTypeDef   txHeader;
	/*State Machine*/
	enum CYBERGEAR_TASK		task;
	/*Motor control*/
	float 								directionCorrection;
	/*Feedback data*/
	union FloatUInt8			realPosDeg;
	union FloatUInt8			realPosRad;
	union FloatUInt8			realVelDeg;
	union FloatUInt8			realVelRad;
	union FloatUInt8			realCur;
	union FloatUInt8			realTorque;
	union FloatUInt8			temperature;
	uint8_t								ifError;
	uint8_t								hallEncoderError;
	uint8_t								ifOverheat;
	uint8_t								ifOvercurrent;
	uint8_t								ifUndervoltage;
	//mode = 0: reset; mode = 1: Calibration; mode = 2: Operation
	uint8_t								mode;
}CybergearHandle;

void CYBERGEAR_Send(CybergearHandle* hmotor, uint8_t cmd, uint16_t data_extended, uint8_t dataLen, uint8_t data[]);
void CYBERGEAR_Enable(CybergearHandle* hmotor);
void CYBERGEAR_Disable(CybergearHandle* hmotor);
void CYBERGEAR_GeneralControl(CybergearHandle* hmotor, float torque, float posRad, float velRad, float kp, float kd);
void CYBERGEAR_GetFeedback(CybergearHandle* hmotor, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[]);
void CYBERGEAR_Zeroing(CybergearHandle* hmotor);
void CYBERGEAR_SetCANIDs(CybergearHandle* hmotor, uint8_t motor_id, uint8_t master_id);

uint8_t CYBERGEAR_CheckCANRxHeader(CybergearHandle* hmotor, CAN_RxHeaderTypeDef rxheader);


CybergearHandle CYBERGEAR_Create(CAN_HandleTypeDef* hcan, uint8_t motor_can_id, uint8_t master_can_id, float direction);



#endif
