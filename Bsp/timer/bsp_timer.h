#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include "bsp_stm32_hal.h"

void bsp_SetTimerFreq(TIM_HandleTypeDef* timer, uint64_t _f_out);
void bsp_SetPWMFreq(TIM_HandleTypeDef* timer, uint32_t channel, uint64_t _f_out, float duty);

#endif //BSP_TIMER_H
