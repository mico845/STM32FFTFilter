#include "bsp_timer.h"
#include <math.h>

void bsp_SetTimerFreq(TIM_HandleTypeDef* timer, uint64_t _f_out)
{
    double Min_PSC_freq;
    double arr_f = 0, psc_f = 0;
    uint64_t _f_in = bsp_GetTimerClock(timer->Instance);

    if (bsp_Timer_GetARR_MAX(timer->Instance) == 16)
        Min_PSC_freq = (double )_f_in / UINT16_MAX;
    else
        Min_PSC_freq = (double )_f_in / UINT32_MAX;

    if(_f_out == 0 || _f_in == 0 || _f_out > _f_in)
        return;
    for (uint16_t i = 1; i < UINT16_MAX; ++i) {
        if ((double )_f_out > Min_PSC_freq / i) {
            psc_f = i;
            arr_f = (double )_f_in/(double )_f_out /i;
            break;
        }
    }
    --arr_f;--psc_f;

    timer->Instance->ARR = (uint32_t)round(arr_f);
    timer->Instance->PSC = (uint32_t)round(psc_f);

}

void bsp_SetPWMFreq(TIM_HandleTypeDef* timer, uint32_t channel, uint64_t _f_out, float duty)
{
    uint32_t CCR;
    bsp_SetTimerFreq(timer, _f_out);
    CCR = roundf(timer->Instance->ARR * duty);
    __HAL_TIM_SET_COMPARE(timer, channel, CCR);
}
