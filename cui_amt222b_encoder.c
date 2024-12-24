#include "cui_amt222b_encoder.h"

uint8_t SetZeroSequence[2] = {0x00, 0x70};
uint8_t EncoderActivateSignal = 0x00;

uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; 	// ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; 	// 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; 						//~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; 						//0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}



/* SPI2 parameter configuration */
/*
hspi->Init.Mode = SPI_MODE_MASTER;
hspi->Init.Direction = SPI_DIRECTION_2LINES;
hspi->Init.DataSize = SPI_DATASIZE_8BIT;
hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
hspi->Init.NSS = SPI_NSS_SOFT;
hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
hspi->Instance->CR1 &= 0xFFFE;//Switch to SPI mode 0
HAL_SPI_Init(hspi);
*/
CUIAMT222BHandle CUI_AMT222b_Create(SPI_HandleTypeDef* hspi, TIM_HandleTypeDef* htim, GPIO_TypeDef* cs_port, uint16_t cs_pin, float angle_offset)
{
	CUIAMT222BHandle hcui;
	hcui.hspi = hspi;
	hcui.htim = htim;
	hcui.csPort = cs_port;
	hcui.csPin = cs_pin;
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	
	hcui.rawRead.b16 = 0;
	hcui.parityErrorCount = 0;
	hcui.successfulReadCount = 0;
  hcui.angleOffset = angle_offset;
  hcui.angleDirectionCorrection = 1.0f;
	
	return hcui;
}

void CUI_AMT222b_Read(CUIAMT222BHandle* hcui)
{
	HAL_GPIO_WritePin(hcui->csPort, hcui->csPin, GPIO_PIN_RESET);
	DWT_Delay_us(10);//  > Tclk = 2.5us
	HAL_SPI_TransmitReceive(hcui->hspi, &EncoderActivateSignal, &(hcui->rawRead).b8[0], 1, 1);
	DWT_Delay_us(10);//  > Tb = 2.5 us
	HAL_SPI_TransmitReceive(hcui->hspi, &EncoderActivateSignal, &(hcui->rawRead).b8[1], 1, 1);
	DWT_Delay_us(10);//  > Tr = 3 us
	HAL_GPIO_WritePin(hcui->csPort, hcui->csPin, GPIO_PIN_SET);
	DWT_Delay_us(100);//  >= Tcs = 40 us
}

void CUI_AMT222b_Get_Angle(CUIAMT222BHandle* hcui)
{
	CUI_AMT222b_Read(hcui);
	
	//Odd parity check
	uint8_t odds = 0, evens = 0;
	uint16_t rawbit16temp = ((uint16_t)hcui->rawRead.b8[1])|((uint16_t)hcui->rawRead.b8[0]<< 8);
	
	//even bits count
	for (uint8_t i = 0; i <=6; i++)
	{
		if ((rawbit16temp >> 2*i) & 0x0001)
			evens++;
	}
	//odd bits count
	rawbit16temp = rawbit16temp >> 1;
	for (uint8_t i = 0; i <=6; i++)
	{
		if ((rawbit16temp >> 2*i) & 0x0001)
			odds++;
	}
	//check validity of even bits
	if ((!(evens%2)) == ((hcui->rawRead.b8[0] >> 6) & 0x01))
		if ((!(odds%2)) == ((hcui->rawRead.b8[0] >> 7) & 0x01))
		{
			hcui->successfulReadCount++;
			hcui->angleBit.b16 = ((uint16_t)hcui->rawRead.b8[1])|((uint16_t)hcui->rawRead.b8[0]<< 8);
			hcui->angleBit.b16 = (hcui->angleBit.b16)&0x3FFF;
			hcui->angleDeg = ((float)hcui->angleBit.b16/16383.0f)*360.0f;
      hcui->angleDeg -= hcui->angleOffset;
      hcui->angleDeg *= hcui->angleDirectionCorrection;
      hcui->angleRad = hcui->angleDeg * deg2rad;
      if(hcui->parityErrorCount)
        hcui->parityErrorCount--;
		}
		else
		{
			hcui->parityErrorCount++;
		}
	else
		hcui->parityErrorCount++;
}

void CUI_AMT222b_Set_ZeroPosition(CUIAMT222BHandle* hcui)
{
	HAL_GPIO_WritePin(hcui->csPort, hcui->csPin, GPIO_PIN_RESET);
	DWT_Delay_us(5);
//  > Tclk = 2.5us
	HAL_SPI_Transmit(hcui->hspi, &SetZeroSequence[0], 1, 1);
	DWT_Delay_us(10);
//  > Tb = 2.5 us
	HAL_SPI_Transmit(hcui->hspi, &SetZeroSequence[1], 1, 1);
	DWT_Delay_us(10);
//  > Tr = 3 us
	HAL_GPIO_WritePin(hcui->csPort, hcui->csPin, GPIO_PIN_SET);
	DWT_Delay_us(50);
//  >= Tcs = 40 us
}
