#ifndef STM32_HAL_H
#define STM32_HAL_H

#define STM32H7xx

#ifdef STM32G4xx
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_it.h"

static inline uint64_t bsp_GetTimerClock(TIM_TypeDef *tim)
{
    return HAL_RCC_GetPCLK1Freq();
}

static inline uint8_t bsp_Timer_GetARR_MAX(TIM_TypeDef *tim)
{
    if (tim == TIM2 || tim == TIM5)
        return 32;
    else
        return 16;
}

#endif //STM32G4xx

#ifdef STM32F4xx
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

static inline uint64_t bsp_GetTimerClock(TIM_TypeDef *tim)
{
    if (tim == TIM1 || tim == TIM8 || tim == TIM9 || tim == TIM10 || tim == TIM11)
        return HAL_RCC_GetPCLK2Freq() * 2;
    else
        return HAL_RCC_GetPCLK1Freq() * 2;
}

static inline uint8_t bsp_Timer_GetARR_MAX(TIM_TypeDef *tim)
{
    if (tim == TIM2 || tim == TIM5)
        return 32;
    else
        return 16;
}

#endif //STM32F4xx

#ifdef STM32H7xx
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_it.h"

#define     __AT_AXI_SRAM_      __attribute__((section("._AXI_SRAM_Area_512KB")))   /**< 将指定对象存放于AXI_SRAM */
#define     __AT_SRAM1_         __attribute__((section("._SRAM1_Area_128KB")))      /**< 将指定对象存放于SRAM1 */
#define     __AT_SRAM2_         __attribute__((section("._SRAM2_Area_128KB")))      /**< 将指定对象存放于SRAM2 */
#define     __AT_SRAM3_         __attribute__((section("._SRAM3_Area_32KB")))       /**< 将指定对象存放于SRAM3 */
#define     __AT_SRAM4_         __attribute__((section("._SRAM4_Area_64KB")))       /**< 将指定对象存放于SRAM4 */
#define     __AT_CPUFLASH_2to8_ __attribute__((section("._CPU_FLASH")))             /**< 将指定对象存放于内部FLASH */

#define     ALIGN_32B(buf)      buf __attribute__ ((aligned (32)))

static inline uint64_t bsp_GetTimerClock(TIM_TypeDef *tim)
{
    if (tim == TIM1 || tim == TIM8 || tim == TIM15 || tim == TIM16 || tim == TIM17)
        return HAL_RCC_GetPCLK2Freq() * 2;
    else
        return HAL_RCC_GetPCLK1Freq() * 2;
}

static inline uint8_t bsp_Timer_GetARR_MAX(TIM_TypeDef *tim)
{
    if (tim == TIM2 || tim == TIM5)
        return 32;
    else
        return 16;
}

#endif //STM32H7xx

#endif //STM32_HAL_H
