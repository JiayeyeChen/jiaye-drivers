#include "embedded_multimeter.h"

void EMBEDDEDMULTIMETER_ReadMeasurements(EmbeddedMultimeterHandle* hmeter, uint16_t mes_reg, uint16_t num_reg)
{
	hmeter->txBuf[0] = hmeter->addr;
	hmeter->txBuf[1] = EMBEDDEDMULTIMETER_CMD_READ_INPUT_REGISTER;
	hmeter->txBuf[2] = (uint8_t)((mes_reg >> 8) & 0xFF);
	hmeter->txBuf[3] = (uint8_t)(mes_reg & 0xFF);
	hmeter->txBuf[4] = (uint8_t)((num_reg >> 8) & 0xFF);
	hmeter->txBuf[5] = (uint8_t)(num_reg & 0xFF);
	uint16_t modbus = CRC16_Modbus(hmeter->txBuf, 6);
	hmeter->txBuf[6] = (uint8_t)(modbus & 0xFF);
	hmeter->txBuf[7] = (uint8_t)((modbus >> 8) & 0xFF);
	HAL_UART_Transmit_DMA(hmeter->huart, hmeter->txBuf, 8);
}

EmbeddedMultimeterHandle EMBEDDEDMULTIMETER_Create(UART_HandleTypeDef* huart, uint8_t addr)
{
	EmbeddedMultimeterHandle hmeter;
	hmeter.huart = huart;
	hmeter.addr = addr;
	hmeter.counts = 0;
	
	HAL_UARTEx_ReceiveToIdle_DMA(huart, hmeter.rxBuf, 50);
	
	return hmeter;
}

void EMBEDDEDMULTIMETER_ReadAllRequest(EmbeddedMultimeterHandle* hmeter)
{
	EMBEDDEDMULTIMETER_ReadMeasurements(hmeter, EMBEDDEDMULTIMETER_REG_VOLTAGE, 8);
}

void EMBEDDEDMULTIMETER_GetData(EmbeddedMultimeterHandle* hmeter)
{
	hmeter->counts++;
	HAL_UART_DMAStop(hmeter->huart);
	
	hmeter->volt.f = (float)((((uint16_t)hmeter->rxBuf[3]) << 8) | ((uint16_t)hmeter->rxBuf[4])) / 100.0f;
	hmeter->curr.f = (float)((((uint16_t)hmeter->rxBuf[5]) << 8) | ((uint16_t)hmeter->rxBuf[6])) / 100.0f;
	hmeter->watt.f = (float)((((uint32_t)hmeter->rxBuf[7]) << 8) | ((uint32_t)hmeter->rxBuf[8]) | (((uint32_t)hmeter->rxBuf[9]) << 24) | (((uint32_t)hmeter->rxBuf[10]) << 16)) / 10.0f;
	hmeter->energy.f = (float)((((uint32_t)hmeter->rxBuf[11]) << 8) | ((uint32_t)hmeter->rxBuf[12]) | (((uint32_t)hmeter->rxBuf[13]) << 24) | (((uint32_t)hmeter->rxBuf[14]) << 16));
	
	HAL_UARTEx_ReceiveToIdle_DMA(hmeter->huart, hmeter->rxBuf, 50);
}
