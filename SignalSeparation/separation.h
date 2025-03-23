#ifndef PROJECT_SEPARATION_H
#define PROJECT_SEPARATION_H

#include "bsp_stm32_hal.h"

#define INTERP_FACTOR 2

#define SAMPLERATE  ((100)*(12.8))
#define DAC_FREQ    (SAMPLERATE * INTERP_FACTOR)


#define ADC_DAC_NUM 128

#ifndef DAC_DMA_BUF_NUM
#define DAC_DMA_BUF_NUM (ADC_DAC_NUM * INTERP_FACTOR)
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

#define __s_arm_cfft_f32(x) arm_cfft_sR_f32_len##x
#define _s_arm_cfft_f32(x) __s_arm_cfft_f32(x)

#define s_arm_cfft_f32 _s_arm_cfft_f32(FFT_NUM)


void signalTask(void);
void process_fft_ifft(void);
void lerp(uint16_t *input, uint16_t *output, uint16_t input_size, uint16_t output_size);

#endif //PROJECT_SEPARATION_H
