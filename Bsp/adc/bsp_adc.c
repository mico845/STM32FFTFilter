#include "bsp_adc.h"
#include "common_inc.h"

#if ADCx==1
ALIGN_32B(uint16_t       adc_dma_buffer[ADC_DMA_BUF_NUM]   __AT_SRAM4_);
#elif ADCx==2
ALIGN_32B(uint32_t       adc_dma_buffer[ADC_DMA_BUF_NUM]   __AT_SRAM4_);
#endif


__IO uint8_t adc_state = 0;

void bsp_Adc_Start(ADC_HandleTypeDef *adc)
{
#if ADCx==1
    HAL_ADC_Start_DMA(adc, (uint32_t *)adc_dma_buffer, ADC_DMA_BUF_NUM );
#elif ADCx==2
    HAL_ADCEx_MultiModeStart_DMA(adc, (uint32_t *)adc_dma_buffer, ADC_DMA_BUF_NUM );
#endif
}

void bsp_Adc_ISR(void)
{
/* 传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_TC) != RESET)
    {
        /*
           1、使用此函数要特别注意，第1个参数地址要32字节对齐，第2个参数要是32字节的整数倍。
           2、进入传输完成中断，当前DMA正在使用缓冲区的前半部分，用户可以操作后半部分。
        */
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[ADC_DMA_BUF_NUM / 2]), ADC_DMA_BUF_NUM);

        adc_state = 2;
        /* 清除标志 */
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_TC;
    }

    /* 半传输完成中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_HC) != RESET)
    {
        /*
           1、使用此函数要特别注意，第1个参数地址要32字节对齐，第2个参数要是32字节的整数倍。
           2、进入半传输完成中断，当前DMA正在使用缓冲区的后半部分，用户可以操作前半部分。
        */
        SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_dma_buffer[0]), ADC_DMA_BUF_NUM);

        adc_state = 1;

        /* 清除标志 */
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_HC;
    }
    /* 传输错误中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_TE) != RESET)
    {
        /* 清除标志 */
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_TE;
    }

    /* 直接模式错误中断 */
    if((ADC_DMAx->LISR & ADC_DMA_Streamx_DME) != RESET)
    {
        /* 清除标志 */
        ADC_DMAx->LIFCR = ADC_DMA_Streamx_DME;
    }
}

bool bsp_Adc_IsFinished(void) {
    if (adc_state == 2) {
        adc_state = 0;
        return true;
    }
    return false;
}