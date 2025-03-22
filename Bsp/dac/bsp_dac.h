#ifndef BSP_DAC_H
#define BSP_DAC_H

#include "bsp_stm32_hal.h"



void bsp_Dac_Start(DAC_HandleTypeDef *dac, uint32_t channel);

void bsp_Dac_DDS_Init(DAC_HandleTypeDef *dac, uint32_t channel, const uint16_t* Buffer, uint32_t Num);
void bsp_Dac_DDS_Config(uint32_t FWORD, uint32_t PWORD);
void bsp_Dac_DDS_SetFreq(uint64_t clkFreq, uint64_t outFreq);
void bsp_Dac_DDS_SetPhase(float Phase);
float bsp_Dac_DDS_GetResolution(uint64_t clkFreq);
void bsp_Dac_DDS_ISR(void);

#endif //BSP_DAC_H
