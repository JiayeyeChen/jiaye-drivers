#include "serial_protocol_echo.h"


SerialProtocolEchoHandle SERIALPROTOCOLECHO_Create(UART_HandleTypeDef* huart)
{
	SerialProtocolEchoHandle hserial;
	hserial.huart = huart;
	hserial.ifNewMsg = 0;
  hserial.invalidRxMsgCount = 0;
	hserial.task = SERIALPROTOCOLECHO_TASK_FREE;
	
	hserial.datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_NA;
	hserial.ifNewDatalogPiece2Send = 0;
	hserial.datalogIndex.b32 = 0;
	hserial.dataSlotLen = 0;
	hserial.datalogLabel2SendPtr = 0;
	hserial.txMsg[0] = 0xAA;
  hserial.txMsg[1] = 0xCC;
	
	return hserial;
}

void SERIALPROTOCOLECHO_TransmitCargo(SerialProtocolEchoHandle* hserial, uint8_t* buf, uint8_t size)
{
  hserial->txLen = size + 6;
  hserial->txMsg[2] = size;
  memcpy(&hserial->txMsg[3], buf, size);
  
  union UInt16UInt8 crc;
  crc.b16 = CRC16_Modbus(hserial->txMsg, size + 3);
  
  hserial->txMsg[size + 3] = crc.b8[0];
  hserial->txMsg[size + 4] = crc.b8[1];
  hserial->txMsg[size + 5] = 0x55;
  HAL_UART_Transmit_DMA(hserial->huart, hserial->txMsg, hserial->txLen);
}

void SERIALPROTOCOLECHO_ReceiveCargoUARTIdleITCallback(SerialProtocolEchoHandle* hserial)
{
  HAL_UART_DMAStop(hserial->huart);//Restart the DMA receiver
  uint8_t i = 0;
//  hserial->dmaPtr  = __HAL_DMA_GET_COUNTER(hserial->huart->hdmarx);
  while (i != 255)//Pointer for scanning through the buffer
  {
    if (hserial->rxMsgRaw[i] == 0xAA && hserial->rxMsgRaw[i + 1] == 0xCC)//Find the start delimiter
    {
      uint8_t tem_size = hserial->rxMsgRaw[i + 2];
      if (tem_size + i > 250)//Msg out of RX buffer range
      {
        hserial->invalidRxMsgCount++;
        break;
      }
      if (hserial->rxMsgRaw[tem_size + 5 + i] == 0x55)//Find the end delimiter
      {
        hserial->rxDataLen = tem_size;
        union UInt16UInt8 temCRC;
        temCRC.b16 = CRC16_Modbus(hserial->rxMsgRaw + i, tem_size + 3);
        if (temCRC.b8[0] == hserial->rxMsgRaw[tem_size + 3 + i] && temCRC.b8[1] == hserial->rxMsgRaw[tem_size + 4 + i])//CRC16 modebus check
        {
          hserial->ifNewMsg = 1;
          memcpy(hserial->rxMsgCfm, hserial->rxMsgRaw + 3 + i, hserial->rxDataLen);
          break;
        }
        else
          hserial->invalidRxMsgCount++;
      }
    }
    i++;
  }
  HAL_UARTEx_ReceiveToIdle_DMA(hserial->huart, hserial->rxMsgRaw, 255);
}

void SERIALPROTOCOLECHO_SendText(SerialProtocolEchoHandle* hserial, char text[])
{
  SERIALPROTOCOLECHO_TransmitCargo(hserial, (uint8_t*)text, strlen(text));
}

uint8_t SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(SerialProtocolEchoHandle* hserial, char str[], uint8_t pos)
{
  if (hserial->ifNewMsg)
  {
    if (!strncmp((const char*)(hserial->rxMsgCfm + pos), (const char*)str, strlen(str)))
    {
      hserial->ifNewMsg = 0;
      return 1;
    }
  }
  return 0;
}

void SERIALPROTOCOLECHO_EnableCommunication(SerialProtocolEchoHandle* hserial)
{
  HAL_UARTEx_ReceiveToIdle_DMA(hserial->huart, hserial->rxMsgRaw, 255);
}

void SERIALPROTOCOLECHO_MasterHost(SerialProtocolEchoHandle* hserial)
{
	/////////////////////////////////////////////////////
	////////////////Check Receive Message////////////////
	/////////////////////////////////////////////////////
	if (hserial->ifNewMsg)
	{
		switch (hserial->task)
    {
    	case SERIALPROTOCOLECHO_TASK_FREE:
				
			if(SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog start", 0))
				SERIALPROTOCOLECHO_StartDatalog(hserial);
    		break;
    	case SERIALPROTOCOLECHO_TASK_DATALOG:
				SERIALPROTOCOLECHO_DatalogManagerReceive(hserial);
    		break;
    	default:
    		break;
    }
	}
	/////////////////////////////////////////////////////
	///////////////////Transmit Message//////////////////
	/////////////////////////////////////////////////////
	switch (hserial->task)
	{
		case SERIALPROTOCOLECHO_TASK_FREE:
			SERIALPROTOCOLECHO_SendText(hserial, "Serial Protocol Echo Master: State Free");
			break;
		case SERIALPROTOCOLECHO_TASK_DATALOG:
			SERIALPROTOCOLECHO_DatalogManagerTransmit(hserial);
			break;
		default:
			break;
	}
}

void SERIALPROTOCOLECHO_DatalogManagerTransmit(SerialProtocolEchoHandle* hserial)
{
	switch (hserial->datalogTask)
  {
  	case SERIALPROTOCOLECHO_DATALOG_TASK_START:
			SERIALPROTOCOLECHO_SendText(hserial, "Datalog start request");
  		break;
  	case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LEN:
			SERIALPROTOCOLECHO_SendDataSlotLen(hserial);
  		break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LABEL:
			hserial->SetLabelFunc();
  		break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_DATA:
			if (hserial->ifNewDatalogPiece2Send)
			{
				SERIALPROTOCOLECHO_DatalogSingleCargoTransmit(hserial, hserial->dataSlot);
				hserial->ifNewDatalogPiece2Send = 0;
			}
			break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_END:
			SERIALPROTOCOLECHO_SendText(hserial, "Datalog end request");
			break;
  	default:
  		break;
  }
}

void SERIALPROTOCOLECHO_DatalogManagerReceive(SerialProtocolEchoHandle* hserial)
{
	switch (hserial->datalogTask)
  {
  	case SERIALPROTOCOLECHO_DATALOG_TASK_START:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog start request received", 0))
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LEN;
  		break;
  	case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LEN:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog length received", 0))
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LABEL;
  		break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_LABEL:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Label received", 0))
				hserial->datalogLabel2SendPtr++;
			if (hserial->datalogLabel2SendPtr >= hserial->dataSlotLen)
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_SEND_DATA;
  		break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_SEND_DATA:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog end", 0))
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_END;
			break;
		case SERIALPROTOCOLECHO_DATALOG_TASK_END:
			if (SERIALPROTOCOLECHO_IfNewMsgAndItIsTheString(hserial, "Datalog end request received", 0))
			{
				hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_NA;
				hserial->task = SERIALPROTOCOLECHO_TASK_FREE;
			}
			break;
  	default:
  		break;
  }
}

void SERIALPROTOCOLECHO_StartDatalog(SerialProtocolEchoHandle* hserial)
{
	hserial->task = SERIALPROTOCOLECHO_TASK_DATALOG;
	hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_START;
	hserial->ifNewDatalogPiece2Send = 0;
	hserial->datalogIndex.b32 = 0;
	hserial->datalogLabel2SendPtr = 0;
	hserial->datalogStartTimestamp = HAL_GetTick();
}

void SERIALPROTOCOLECHO_SetNewDatalogSlotLength(SerialProtocolEchoHandle* hserial, uint8_t len)
{
	hserial->dataSlotLen = len;
}

void SERIALPROTOCOLECHO_SendDataSlotLen(SerialProtocolEchoHandle* hserial)
{
  char numStr[2];
  char txStr[7];
  strncpy(txStr, "len: ", 5);
  int numStrLen;
  numStrLen = sprintf(numStr, "%d", hserial->dataSlotLen);
  strncpy(&txStr[5], numStr, numStrLen);
  SERIALPROTOCOLECHO_TransmitCargo(hserial, (uint8_t*)txStr, numStrLen + 5);
}

void SERIALPROTOCOLECHO_SendDataSlotLabel(SerialProtocolEchoHandle* hserial, char* label_1, ...)
{
	va_list label_ptr;
  va_start(label_ptr, label_1);
 
  uint8_t numOfLabels = atoi(label_1);
  for (uint8_t i = 0; i < numOfLabels; i++)
  {
    char buf[50];
    buf[0] = i + 1;
    strcpy(&buf[1], va_arg(label_ptr, char*));
    if (i == hserial->datalogLabel2SendPtr)
    {
      SERIALPROTOCOLECHO_TransmitCargo(hserial, (uint8_t*)buf, strlen(buf));
      break;
    }
  }
  va_end(label_ptr);
}

void SERIALPROTOCOLECHO_SetNewDatalogSendLabelFunction(SerialProtocolEchoHandle* hserial, FuncTypeVoidVoid func)
{
	hserial->SetLabelFunc = func;
}

void SERIALPROTOCOLECHO_SetNewDatalogSlot(SerialProtocolEchoHandle* hserial, union FloatUInt8 (*data_slot))
{
	hserial->dataSlot = data_slot;
}

void SERIALPROTOCOLECHO_DatalogSingleCargoTransmit(SerialProtocolEchoHandle* hserial, union FloatUInt8 data_slots[])
{
	uint8_t i = 0;
  union UInt32UInt8 sysTick;
  sysTick.b32 = HAL_GetTick() - hserial->datalogStartTimestamp;
  //Index
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[0];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[1];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[2];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[3];
  hserial->datalogIndex.b32++;
  //Time stamp
  hserial->datalogBuf[i++] = sysTick.b8[0];
  hserial->datalogBuf[i++] = sysTick.b8[1];
  hserial->datalogBuf[i++] = sysTick.b8[2];
  hserial->datalogBuf[i++] = sysTick.b8[3];
  //Data
  for (uint8_t j = 0; j < hserial->dataSlotLen; j++)
  {
    hserial->datalogBuf[i++] = data_slots[j].b8[0];
    hserial->datalogBuf[i++] = data_slots[j].b8[1];
    hserial->datalogBuf[i++] = data_slots[j].b8[2];
    hserial->datalogBuf[i++] = data_slots[j].b8[3];
  }
  SERIALPROTOCOLECHO_TransmitCargo(hserial, hserial->datalogBuf, hserial->dataSlotLen * 4 + 8);
}

void SERIALPROTOCOLECHO_EndDatalog(SerialProtocolEchoHandle* hserial)
{
	hserial->datalogTask = SERIALPROTOCOLECHO_DATALOG_TASK_END;
}
