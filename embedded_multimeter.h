#ifndef EMBEDDED_MULTIMETER_H
#define EMBEDDED_MULTIMETER_H

#include "common.h"


#define EMBEDDEDMULTIMETER_CMD_READ_RESERVE_REGISTER 0x03
#define EMBEDDEDMULTIMETER_CMD_READ_INPUT_REGISTER   0x04
#define EMBEDDEDMULTIMETER_CMD_WRITE_REGISTER				 0x06
#define EMBEDDEDMULTIMETER_CMD_RESET_ENERGYMETER		 0x42

#define EMBEDDEDMULTIMETER_REG_VOLTAGE							 0x0000
#define EMBEDDEDMULTIMETER_REG_CURRENT							 0x0001
#define EMBEDDEDMULTIMETER_REG_WATT_LSB							 0x0002
#define EMBEDDEDMULTIMETER_REG_WATT_MSB							 0x0003
#define EMBEDDEDMULTIMETER_REG_ENERGY_LSB						 0x0004
#define EMBEDDEDMULTIMETER_REG_ENERGY_MSB						 0x0005
#define EMBEDDEDMULTIMETER_REG_HIGH_VOLTAGE_ALARM		 0x0006
#define EMBEDDEDMULTIMETER_REG_LOW_VOLTAGE_ALARM     0x0007






typedef struct
{
	UART_HandleTypeDef*   huart;
	uint8_t 							addr;
	uint8_t								txBuf[8];
	uint8_t								rxBuf[50];
	uint32_t							counts;
	
	union FloatUInt8			volt;
	union FloatUInt8			curr;
	union FloatUInt8			watt;
	union FloatUInt8			energy;
}EmbeddedMultimeterHandle;

EmbeddedMultimeterHandle EMBEDDEDMULTIMETER_Create(UART_HandleTypeDef* huart, uint8_t addr);

void EMBEDDEDMULTIMETER_ReadMeasurements(EmbeddedMultimeterHandle* hmeter, uint16_t mes_reg, uint16_t num_reg);
void EMBEDDEDMULTIMETER_ReadAllRequest(EmbeddedMultimeterHandle* hmeter);
void EMBEDDEDMULTIMETER_GetData(EmbeddedMultimeterHandle* hmeter);
















#endif
