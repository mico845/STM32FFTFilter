#include "vofa_debug.h"
#include "uart/retarget.h"

void Debug_u16(const uint16_t * signal, uint32_t num)
{
    for (uint32_t i= 0; i < num; ++i) {
        printf("%lu,%hu\r\n", i, signal[i]);
    }
}

void Debug_u32(uint32_t * signal, uint32_t num)
{
    for (uint32_t i= 0; i < num; ++i) {
        printf("%lu,%lu\r\n",i,signal[i]);
    }
}

void Debug_f32(float32_t* signal, uint32_t num)
{
    for (uint32_t i= 0; i < num; ++i) {
        printf("%lu,%f\r\n",i,signal[i]);
    }
}

