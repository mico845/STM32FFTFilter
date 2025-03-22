#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_stm32_hal.h"
#include <stdbool.h>



#define ADC_DMAx            DMA2

#define ADC_DMA_Streamx_TC      DMA_FLAG_TCIF0_4
#define ADC_DMA_Streamx_HC      DMA_FLAG_HTIF0_4
#define ADC_DMA_Streamx_TE      DMA_FLAG_TEIF0_4
#define ADC_DMA_Streamx_DME     DMA_FLAG_DMEIF0_4

extern __IO uint8_t adc_state;

void bsp_Adc_Start(ADC_HandleTypeDef *adc);
void bsp_Adc_ISR(void);
bool bsp_Adc_IsFinished(void);

#endif //BSP_ADC_H
