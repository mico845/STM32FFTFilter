#include "separation.h"
#include "common_inc.h"

float32_t bufferA[DAC_DMA_BUF_NUM];
float32_t bufferB[DAC_DMA_BUF_NUM];

volatile bool active_buffer = true;

void smooth(uint16_t *data, uint16_t size);

void signalTask(void)
{
    /* 半传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_HC) != RESET)
    {
        float32_t *target_buffer = (active_buffer) ? bufferA : bufferB;
        float32_t *processing_buffer = (active_buffer) ? bufferB : bufferA;
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[0]), ADC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < ADC_DMA_BUF_NUM / 2; ++i) {
            target_buffer[i] = adc_dma_buffer[i];

        }
        for (uint16_t i = 0; i < DAC_DMA_BUF_NUM / 2; ++i) {
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

        for (uint16_t i = 0; i < ADC_DMA_BUF_NUM / 2; ++i) {
            target_buffer[i + ADC_DMA_BUF_NUM / 2] = adc_dma_buffer[i + ADC_DMA_BUF_NUM / 2];

        }
        for (uint16_t i = 0; i < DAC_DMA_BUF_NUM / 2; ++i) {
            dac_dma_buf[i + DAC_DMA_BUF_NUM / 2] = (uint16_t)processing_buffer[i + DAC_DMA_BUF_NUM / 2];
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

        arm_cfft_f32(&s_arm_cfft_f32, fft_input, 0, 1);

        uint16_t cutoff_bin = (uint16_t)((120.0 / SAMPLERATE) * (FFT_NUM * 2));

        for (uint16_t i = cutoff_bin; i < FFT_NUM/2; i++) {
            float32_t window_coeff = 0.22 * (1 + cosf(M_PI * (i - cutoff_bin) / (FFT_NUM/2 - cutoff_bin)));
            fft_input[2 * i] *= window_coeff;
            fft_input[2 * i + 1] *= window_coeff;
            fft_input[2 * (FFT_NUM - i)] *= window_coeff;
            fft_input[2 * (FFT_NUM - i) + 1] *= window_coeff;
        }


//#if DEBUG_FFT
//        if (debug_times == 5)
//        {
////          幅频响应
////          arm_cmplx_mag_f32(fft_input, signal_freq, FFT_NUM);
////          Debug_f32(signal_freq, FFT_NUM);
//
//
////          FFT 复数原始结果
//            Debug_f32(fft_input, FFT_NUM*2);
//
//            debug_times = 99;
//        }
//        else if (debug_times <= 11)
//            debug_times++;
//#endif

        arm_cfft_f32(&s_arm_cfft_f32, fft_input, 1, 1);

        uint16_t interpolated_output[DAC_DMA_BUF_NUM];
        uint16_t interpolated_input[FFT_NUM];

        for (uint16_t i = 0; i < FFT_NUM; i++) {
            interpolated_input[i] = fft_input[2 * i];
        }

        lerp(interpolated_input, interpolated_output, FFT_NUM, DAC_DMA_BUF_NUM);

        for (uint16_t i = 0; i < DAC_DMA_BUF_NUM; i++) {
            processing_buffer[i] = interpolated_output[i];
        }


#if DEBUG_FFT
        if (debug_times == 5)
        {
            Debug_u16(interpolated_output, DAC_DMA_BUF_NUM);
            debug_times = 99;
        }
        else if (debug_times <= 11)
            debug_times++;
#endif
        active_buffer = !active_buffer;
        adc_state = 0;
    }
}


void lerp(uint16_t *input, uint16_t *output, uint16_t input_size, uint16_t output_size)
{
    uint16_t w = output_size / input_size;
    float index;
    uint16_t idx;
    float frac;

    for (uint16_t i = 0; i < output_size; i++)
    {
        index = (float)i / w;
        idx = (uint16_t)index;
        frac = index - idx;

        if (idx < input_size - 1)
        {
            output[i] = (uint16_t)((1 - frac) * input[idx] + frac * input[idx + 1]);
        }
        else
        {
            output[i] = input[input_size - 1];
        }
    }
}


void smooth(uint16_t *data, uint16_t size)
{
    for (uint16_t i = 1; i < size - 1; i++)
    {
        data[i] = (data[i - 1] + data[i] + data[i + 1]) / 3;
    }
}
