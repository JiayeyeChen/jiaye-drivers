#include "ad7606.h"

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
												SPI_HandleTypeDef* hspi)
{
	AD7606Handle hadc;
	hadc.busy_gpio_port = busy_gpio_port;
	hadc.busy_gpio_pin = busy_gpio_pin;
	hadc.conv_gpio_port = conv_gpio_port;
	hadc.conv_gpio_pin = conv_gpio_pin;
	hadc.cs_gpio_port = cs_gpio_port;
	hadc.cs_gpio_pin = cs_gpio_pin;
	hadc.os0_gpio_port = os0_gpio_port;
	hadc.os0_gpio_pin = os0_gpio_pin;
	hadc.os1_gpio_port = os1_gpio_port;
	hadc.os1_gpio_pin = os1_gpio_pin;
	hadc.os2_gpio_port = os2_gpio_port;
	hadc.os2_gpio_pin = os2_gpio_pin;
	hadc.range_gpio_port = range_gpio_port;
	hadc.range_gpio_pin = range_gpio_pin;
	hadc.rst_gpio_port = rst_gpio_port;
	hadc.rst_gpio_pin = rst_gpio_pin;
	hadc.hspi = hspi;
	
	HAL_GPIO_WritePin(hadc.cs_gpio_port, hadc.cs_gpio_pin, GPIO_PIN_SET);
	//Delay for some time
	HAL_GPIO_WritePin(hadc.cs_gpio_port, hadc.cs_gpio_pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(hadc.rst_gpio_port, hadc.rst_gpio_pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(hadc.rst_gpio_port, hadc.rst_gpio_pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(hadc.rst_gpio_port, hadc.rst_gpio_pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	
	return hadc;
}

void AD7606_DataRequest(AD7606Handle* hadc)
{
	HAL_GPIO_WritePin(hadc->conv_gpio_port, hadc->conv_gpio_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(hadc->conv_gpio_port, hadc->conv_gpio_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(hadc->conv_gpio_port, hadc->conv_gpio_pin, GPIO_PIN_RESET);
}

void AD7606_ReadRawData(AD7606Handle* hadc)
{
	//t4 = 0 ns
	HAL_GPIO_WritePin(hadc->cs_gpio_port, hadc->cs_gpio_pin, GPIO_PIN_RESET);
  HAL_SPI_Receive(hadc->hspi, (uint8_t *)(hadc->rawData), 8, 10);
	HAL_GPIO_WritePin(hadc->cs_gpio_port, hadc->cs_gpio_pin, GPIO_PIN_SET);
}

void AD7606_GetVoltage(AD7606Handle* hadc)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    hadc->volt[i] = ((float)hadc->rawData[i]) * 5.0f / 32767.0f;
  }
}
