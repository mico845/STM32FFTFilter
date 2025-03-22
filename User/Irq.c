#include "common_inc.h"

extern DMA_HandleTypeDef hdma_dac1_ch1;

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_dac1_ch1);
}



void DMA2_Stream0_IRQHandler(void)
{
    signalTask();
}
