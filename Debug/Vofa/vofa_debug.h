#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include "arm_math.h"

void Debug_u16(const uint16_t * signal, uint32_t num);
void Debug_u32(uint32_t * signal, uint32_t num);
void Debug_f32(float32_t* signal, uint32_t num);

#endif //VOFA_DEBUG_H
