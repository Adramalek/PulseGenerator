#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include <string.h>
#include <stdlib.h>

#define BAUD_RATE 9600
#define CMD_MAX_LEN 14
#define END_CHAR 'x'

typedef enum {
	SINGLE = 0,
	MULTI = !SINGLE
} Impulse_Mode;

typedef struct {
	uint16_t Prescaler;
	uint16_t Period;
	uint16_t TIM_CounterMode;
	TIM_TypeDef* TIM;
	uint32_t RCC_APB1Periph_TIM;
	uint8_t NVIC_TIM_IRQChannel;
} TIM_Data;

void Start_Timers(void);
void Stop_Timers(void);
uint8_t Rescale(uint8_t);
void Set_Autoreload(uint8_t);
void Set_Delay(uint8_t);
void Set_Pulse(uint8_t);
void Set_Pulse_Delay(uint8_t); // this is not an union of two functions above
uint8_t Parse_Apply_Cmd(char*);
void Send_Message(char *);
void ReinitDelays(void);
void Init(void);
void Init_Pre(void);
void Init_Post(void);
void Init_Delay(void);
void Init_Pulse_Delay(void);
void Init_USART(void);
void Init_LED(void);
void Init_Button_Interrupt(void);

#endif
