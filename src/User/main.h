#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include "defines.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_exti.h"

#define DELAY 12
#define TRAINING_TIME 120



typedef struct {
	uint16_t Prescaler;
	uint16_t Period;
	TIM_TypeDef* TIM;
	uint32_t RCC_APB1Periph_TIM;
	uint8_t NVIC_TIM_IRQChannel;
} TIM_Data;

#endif
