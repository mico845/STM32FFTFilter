#ifndef PROJECT_SEPARATION_H
#define PROJECT_SEPARATION_H

#include "bsp_stm32_hal.h"


#define SAMPLERATE  ((100)*(12.8))

#define ADC_DAC_NUM 128

#ifndef DAC_DMA_BUF_NUM
#define DAC_DMA_BUF_NUM ADC_DAC_NUM
#endif

extern uint16_t dac_dma_buf[DAC_DMA_BUF_NUM];

#define ADC_DMA_BUF_NUM ADC_DAC_NUM

#define ADCx 1
#if ADCx==1
extern uint16_t       adc_dma_buffer[ADC_DMA_BUF_NUM];
#elif ADCx==2
extern uint32_t       adc_dma_buffer[ADC_DMA_BUF_NUM];
#endif

#define FFT_NUM ADC_DAC_NUM

void signalTask(void);
void process_fft_ifft(void);

#endif //PROJECT_SEPARATION_H
