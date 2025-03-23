#include "common_inc.h"

#define DEBUG_ADC_DMA 0

_Noreturn void Main(void)
{
    RetargetInit(&huart1);
    spline_Init();

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    bsp_Adc_Start(&hadc1);


    bsp_SetTimerFreq(&htim1, SAMPLERATE);
    HAL_TIM_Base_Start_IT(&htim1);


    bsp_SetTimerFreq(&htim8, DAC_FREQ);
    HAL_TIM_Base_Start_IT(&htim8);

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_dma_buf, DAC_DMA_BUF_NUM, DAC_ALIGN_12B_R);
    HAL_OPAMP_Start(&hopamp1);


    while (true)
    {
        process_fft_ifft();
    }
}