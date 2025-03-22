#ifndef PROJECT_COMMON_INC_H
#define PROJECT_COMMON_INC_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/

/* BSP */
#include "uart/retarget.h"
#include "adc/bsp_adc.h"
#include "timer/bsp_timer.h"
#include "dac/bsp_dac.h"

/* Lib */
#include "arm_math.h"
#include "arm_const_structs.h"

/* Debug */
#include "Vofa/vofa_debug.h"

/* STD */
#include "stdbool.h"

/* HAL */
#include "main.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "opamp.h"

/* Task */
#include "separation.h"


_Noreturn void Main(void);

#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/
#endif



#endif //PROJECT_COMMON_INC_H
