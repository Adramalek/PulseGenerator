#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include "defines.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_exti.h"

typedef enum {
	SINGLE,
	MULTI
} Impulse_Mode;

Impulse_Mode Mode = MULTI;

typedef struct {
	uint16_t Prescaler;
	uint16_t Period;
	uint16_t TIM_CounterMode;
	TIM_TypeDef* TIM;
	uint32_t RCC_APB1Periph_TIM;
	uint8_t NVIC_TIM_IRQChannel;
} TIM_Data;

inline void Start_Timers(void);
inline void Stop_Timers(void);
#endif
