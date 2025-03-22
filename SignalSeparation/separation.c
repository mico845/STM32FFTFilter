#include "separation.h"
#include "common_inc.h"

float32_t bufferA[FFT_NUM];
float32_t bufferB[FFT_NUM];

volatile bool active_buffer = true;

void signalTask(void)
{
    /* 半传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_HC) != RESET)
    {
        float32_t *target_buffer = (active_buffer) ? bufferA : bufferB;
        float32_t *processing_buffer = (active_buffer) ? bufferB : bufferA;
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[0]), ADC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < FFT_NUM / 2; ++i) {
            target_buffer[i] = adc_dma_buffer[i];
            dac_dma_buf[i] = (uint16_t)processing_buffer[i];
        }
        adc_state = 1;
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_HC;
    }

    /* 传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_TC) != RESET)
    {
        float32_t *target_buffer = (active_buffer) ? bufferA : bufferB;
        float32_t *processing_buffer = (active_buffer) ? bufferB : bufferA;
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[ADC_DMA_BUF_NUM / 2]), ADC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < FFT_NUM / 2; ++i) {
            target_buffer[i + FFT_NUM / 2] = adc_dma_buffer[i + FFT_NUM / 2];
            dac_dma_buf[i + FFT_NUM / 2] = (uint16_t)processing_buffer[i + FFT_NUM / 2];
        }

        adc_state = 2;
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_TC;
    }
}

float32_t fft_input[FFT_NUM*2];


#define DEBUG_FFT (1)
#if DEBUG_FFT
uint8_t debug_times = 0;
float32_t signal_freq[FFT_NUM];
#endif

void process_fft_ifft(void)
{
    if (adc_state == 2)
    {
        float32_t *processing_buffer = (active_buffer) ? bufferA : bufferB;
        for (uint16_t i = 0; i < FFT_NUM; ++i)
        {
            fft_input[i * 2] = processing_buffer[i];
            fft_input[i * 2 + 1] = 0;
        }

        arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_input, 0, 1);

        uint16_t cutoff_bin = (uint16_t)((120.0 / SAMPLERATE) * (FFT_NUM * 2));

        for (uint16_t i = cutoff_bin; i < FFT_NUM/2; i++) {
            float window_coeff = 0.22 * (1 + cosf(M_PI * (i - cutoff_bin) / (FFT_NUM/2 - cutoff_bin))); // Hanning 窗
            fft_input[2 * i] *= window_coeff;
            fft_input[2 * i + 1] *= window_coeff;
            fft_input[2 * (FFT_NUM - i)] *= window_coeff;
            fft_input[2 * (FFT_NUM - i) + 1] *= window_coeff;
        }


#if DEBUG_FFT
        if (debug_times == 5)
        {
//          幅频响应
//          arm_cmplx_mag_f32(fft_input, signal_freq, FFT_NUM);
//          Debug_f32(signal_freq, FFT_NUM);


//          FFT 复数原始结果
            Debug_f32(fft_input, FFT_NUM*2);

            debug_times = 99;
        }
        else if (debug_times <= 11)
            debug_times++;
#endif



        arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_input, 1, 1);

        for (uint16_t i = 0; i < FFT_NUM; i++) {
            processing_buffer[i] = fft_input[i * 2];
        }
        
        active_buffer = !active_buffer;
        adc_state = 0;
    }
}