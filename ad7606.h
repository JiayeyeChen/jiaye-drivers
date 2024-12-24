#ifndef AD7606_H
#define AD7606_H

#include "common.h"

#define AD7606_OS_RATIO_NONE		0x00  //Max CONVST Freq = 200   kHz
#define AD7606_OS_RATIO_2				0x01  //Max CONVST Freq = 100   kHz
#define AD7606_OS_RATIO_4				0x02  //Max CONVST Freq = 50    kHz
#define AD7606_OS_RATIO_8				0x03  //Max CONVST Freq = 25    kHz
#define AD7606_OS_RATIO_16			0x04  //Max CONVST Freq = 12.5  kHz
#define AD7606_OS_RATIO_32			0x05  //Max CONVST Freq = 6.25  kHz
#define AD7606_OS_RATIO_64			0x06  //Max CONVST Freq = 3.125 kHz
#define AD7606_OS_RATIO_NA			0xFF

typedef struct
{
  SPI_HandleTypeDef*      hspi;
	GPIO_TypeDef* os0_gpio_port;
	uint16_t os0_gpio_pin;
	GPIO_TypeDef* os1_gpio_port;
	uint16_t os1_gpio_pin;
	GPIO_TypeDef* os2_gpio_port;
	uint16_t os2_gpio_pin;
	GPIO_TypeDef* range_gpio_port;
	uint16_t range_gpio_pin;
	GPIO_TypeDef* conv_gpio_port;
	uint16_t conv_gpio_pin;
	GPIO_TypeDef* rst_gpio_port;
	uint16_t rst_gpio_pin;
	GPIO_TypeDef* cs_gpio_port;
	uint16_t cs_gpio_pin;
	GPIO_TypeDef* busy_gpio_port;
	uint16_t busy_gpio_pin;
	
	
  int16_t                 rawData[8];
  float                   volt[8];
}AD7606Handle;

void AD7606_Init(void);
AD7606Handle AD7606_Create(uint8_t oversampling_rate, \
												GPIO_TypeDef* os0_gpio_port, uint16_t os0_gpio_pin, \
												GPIO_TypeDef* os1_gpio_port, uint16_t os1_gpio_pin, \
												GPIO_TypeDef* os2_gpio_port, uint16_t os2_gpio_pin,\
												uint8_t range,\
												GPIO_TypeDef* range_gpio_port, uint16_t range_gpio_pin,\
												GPIO_TypeDef* conv_gpio_port, uint16_t conv_gpio_pin,\
												GPIO_TypeDef* rst_gpio_port, uint16_t rst_gpio_pin,\
												GPIO_TypeDef* cs_gpio_port, uint16_t cs_gpio_pin,\
												GPIO_TypeDef* busy_gpio_port, uint16_t busy_gpio_pin,\
												SPI_HandleTypeDef* hspi);
void AD7606_Reset(AD7606Handle* hadc);

void AD7606_DataRequest(AD7606Handle* hadc);

void AD7606_ReadRawData(AD7606Handle* hadc);
void AD7606_GetVoltage(AD7606Handle* hadc);

#endif
