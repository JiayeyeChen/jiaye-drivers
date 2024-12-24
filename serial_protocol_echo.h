#ifndef SERIAL_PROTOCOL_ECHO_H
#define SERIAL_PROTOCOL_ECHO_H

#include "common.h"

enum SERIALPROTOCOLECHO_Task
{
  SERIALPROTOCOLECHO_TASK_FREE,
	SERIALPROTOCOLECHO_TASK_DATALOG,
};

enum SERIALPROTOCOLECHO_DatalogTask
{
	SERIALPROTOCOLECHO_DATALOG_TASK_NA,
  SERIALPROTOCOLECHO_DATALOG_TASK_START,
	SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LEN,
	SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LABEL,
	SERIALPROTOCOLECHO_DATALOG_TASK_SEND_DATA,
	SERIALPROTOCOLECHO_DATALOG_TASK_END
};

typedef struct
{
  UART_HandleTypeDef*   huart;
  /* Receive */
  uint8_t               rxMsgRaw[255];
  uint8_t               rxMsgCfm[255];
  uint8_t               rxDataLen;//Max byte number = 255 - 6 = 249
  uint8_t               ifNewMsg;
  uint32_t              invalidRxMsgCount;
  uint32_t              dmaPtr;
  /* Transmit */
  uint8_t               txMsg[255];
  uint8_t               txLen;
	/* Task schedule */
	enum SERIALPROTOCOLECHO_Task task;
  /* Datalog */
	enum SERIALPROTOCOLECHO_DatalogTask datalogTask;
  union FloatUInt8*     dataSlot;
  void                  (*SetLabelFunc) (void);
  uint8_t               ifNewDatalogPiece2Send;
  union UInt32UInt8     datalogIndex;
  uint8_t               dataSlotLen;
  uint32_t              datalogStartTimestamp;
  uint8_t               datalogLabel2SendPtr;
  uint8_t               datalogBuf[255];
}SerialProtocolEchoHandle;

SerialProtocolEchoHandle SERIALPROTOCOLECHO_Create(UART_HandleTypeDef* huart);
void SERIALPROTOCOLECHO_TransmitCargo(SerialProtocolEchoHandle* hserial, uint8_t* buf, uint8_t size);
void SERIALPROTOCOLECHO_ReceiveCargoUARTIdleITCallback(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_SendText(SerialProtocolEchoHandle* hserial, char text[]);
uint8_t SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(SerialProtocolEchoHandle* hserial, char str[], uint8_t pos);
void SERIALPROTOCOLECHO_MasterHost(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_EnableCommunication(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_DatalogManagerTransmit(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_DatalogManagerReceive(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_DatalogSingleCargoTransmit(SerialProtocolEchoHandle* hserial, union FloatUInt8 data_slots[]);
void SERIALPROTOCOLECHO_StartDatalog(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_EndDatalog(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_SetNewDatalogSlotLength(SerialProtocolEchoHandle* hserial, uint8_t len);
void SERIALPROTOCOLECHO_SendDataSlotLen(SerialProtocolEchoHandle* hserial);
void SERIALPROTOCOLECHO_SendDataSlotLabel(SerialProtocolEchoHandle* hserial, char* label_1, ...);
void SERIALPROTOCOLECHO_SetNewDatalogSendLabelFunction(SerialProtocolEchoHandle* hserial, FuncTypeVoidVoid func);
void SERIALPROTOCOLECHO_SetNewDatalogSlot(SerialProtocolEchoHandle* hserial, union FloatUInt8 (*data_slot));
#endif
