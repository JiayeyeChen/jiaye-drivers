#ifndef CUI_AMT222B_ENCODER_H
#define CUI_AMT222B_ENCODER_H

#include "common.h"

typedef struct
{
	SPI_HandleTypeDef*    hspi;                /* The SPI port for this instance. */
	union UInt16UInt8			angleBit;
	float									angleDeg;
  float                 angleRad;
	float                 angleOffset;
  float                 angleDirectionCorrection;
	union UInt16UInt8			rawRead;
	GPIO_TypeDef*         csPort;
	uint16_t              csPin;
	uint32_t							parityErrorCount;
	uint32_t							successfulReadCount;
}CUIAMT222BHandle;


CUIAMT222BHandle CUI_AMT222b_Create(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, float angle_offset);

uint32_t DWT_Delay_Init(void);

void CUI_AMT222b_Read(CUIAMT222BHandle* hcui);


void CUI_AMT222b_Get_Angle(CUIAMT222BHandle* hcui);


void CUI_AMT222b_Set_ZeroPosition(CUIAMT222BHandle* hcui);


__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

#endif
