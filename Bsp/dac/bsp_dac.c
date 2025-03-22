#include "bsp_dac.h"
#include "common_inc.h"

ALIGN_32B(uint16_t dac_dma_buf[DAC_DMA_BUF_NUM]  __AT_SRAM4_);

typedef struct
{
    uint32_t F_WORD;    //频率控制字
    uint32_t P_WORD;    //相位控制字

    uint32_t ROM_Index;
    const uint16_t *pROM;
    uint32_t ROM_Num;
}DDS_T;

DDS_T dac_dds;

void bsp_Dac_Start(DAC_HandleTypeDef *dac, uint32_t channel)
{
    HAL_DAC_Start_DMA(dac, channel, (uint32_t *)dac_dma_buf, DAC_DMA_BUF_NUM, DAC_ALIGN_12B_R);
}

void bsp_Dac_DDS_Init(DAC_HandleTypeDef *dac, uint32_t channel, const uint16_t* Buffer, uint32_t Num)
{
    dac_dds.pROM = Buffer;
    dac_dds.ROM_Num = Num;
    dac_dds.F_WORD = 0;    //频率控制字
    dac_dds.P_WORD = 0;    //相位控制字
    dac_dds.ROM_Index = 0;
    HAL_DAC_Start_DMA(dac, channel, (uint32_t *)dac_dma_buf, DAC_DMA_BUF_NUM, DAC_ALIGN_12B_R);
}

void bsp_Dac_DDS_Config(uint32_t FWORD, uint32_t PWORD)
{
    dac_dds.F_WORD = FWORD;    //频率控制字
    dac_dds.P_WORD = PWORD;    //相位控制字
}

void bsp_Dac_DDS_SetFreq(uint64_t clkFreq, uint64_t outFreq)
{
    dac_dds.F_WORD = outFreq * dac_dds.ROM_Num / clkFreq;
}

void bsp_Dac_DDS_SetPhase(float Phase)
{
    dac_dds.P_WORD = roundf((float )Phase / (float )dac_dds.ROM_Num * 360.0);
}

float bsp_Dac_DDS_GetResolution(uint64_t clkFreq)
{
    return (float)clkFreq/(float)dac_dds.ROM_Num;
}

void bsp_Dac_DDS_ISR(void)
{
    for(uint32_t i = 0; i < (DAC_DMA_BUF_NUM / 2); i++)
    {
        dac_dma_buf[dac_dds.ROM_Index] = dac_dds.pROM[dac_dds.P_WORD];
        dac_dds.ROM_Index++;

        dac_dds.P_WORD+=dac_dds.F_WORD;
        if (dac_dds.P_WORD >= dac_dds.ROM_Num)  //仅溢出后才进行取模运算，优化时间
            dac_dds.P_WORD %= dac_dds.ROM_Num;

        if (dac_dds.ROM_Index == DAC_DMA_BUF_NUM)
            dac_dds.ROM_Index = 0;
    }
}


